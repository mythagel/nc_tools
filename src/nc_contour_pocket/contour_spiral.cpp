#include "rs274ngc_return.hh"
#include <boost/program_options.hpp>
#include "print_exception.h"
#include "../throw_if.h"
#include "rs274_clipper_path.h"
#include <iostream>
#include "clipper.hpp"
#include "base/machine_config.h"
#include <algorithm>
#include "../r6.h"
#include "common.h"

namespace po = boost::program_options;
namespace cl = ClipperLib;
using namespace geometry;

struct Node {
    ClipperLib::Paths polygon;
    std::vector<Node> children;
};

void fold_path (const cl::PolyNode* node, cl::Paths& paths) {
    paths.push_back(node->Contour);
    for (auto& child : node->Childs)
        fold_path(child, paths);
}

void offset_node(Node& node, double offset, double scale, unsigned level = 0) {
    if (node.polygon.empty())
        return;

    cl::ClipperOffset co;
    co.AddPaths(node.polygon, cl::jtRound, cl::etClosedPolygon);
    co.ArcTolerance = 0.025 * scale;

    cl::Paths solution;
    co.Execute(solution, offset * scale);
    if (solution.empty())
        return;

    // Reduce the number of points in the result
    cl::CleanPolygons(solution, 1e-5*scale);

    {
        cl::Clipper clipper;
        clipper.AddPaths(solution, cl::ptSubject, true);

        cl::PolyTree pt;
        clipper.Execute(cl::ctUnion, pt);

        node.children.reserve(pt.Childs.size());
        for (auto& child : pt.Childs) {
            node.children.emplace_back();
            Node& cn = node.children.back();

            fold_path(child, cn.polygon);
            offset_node(cn, offset, scale, level+1);
        }
    }
}

void segment(Node node, std::vector<Node>& nodes) {
    Node flat;

    Node* curr = &node;
    while (curr) {
        if (curr->children.size() == 1) {
            flat.children.push_back({ curr->polygon, {} });
            curr = &curr->children.back();
        } else if (curr->children.size() == 0) {
            flat.children.push_back({ curr->polygon, {} });
            nodes.push_back(flat);
            curr = nullptr;
        } else {
            flat.children.push_back({ curr->polygon, {} });
            nodes.push_back(flat);

            for (auto& node : curr->children) {
                segment(node, nodes);
            }
            curr = nullptr;
        }
    }
}

// debug
void output_node(const Node& node, double z, double scale, unsigned level, bool children = true) {
    auto unscale_point = [&](const cl::IntPoint& p) -> point_2 {
        return {static_cast<double>(p.X)/scale, static_cast<double>(p.Y)/scale};
    };

    for (auto& path : node.polygon) {
        bool rapid_to_first = true;
        for (auto& point : path) {
            auto p = unscale_point(point);
            if (rapid_to_first) {
                std::cout << "G0 X" << r6(p.x) << " Y" << r6(p.y) << "\n";
                std::cout << "G1 Z" << r6(z+level) << " F" << r6(50) << "\n";
                rapid_to_first = false;
            }
            std::cout << "   X" << r6(p.x) << " Y" << r6(p.y) << "\n";
        }
        auto p = unscale_point(*path.begin());
        std::cout << "   X" << r6(p.x) << " Y" << r6(p.y) << "\n";
    }

    if (children) {
        for (auto& child : node.children) {
            output_node(child, z, scale, level+1);
        }
    }
}

int main(int argc, char* argv[]) {
    po::options_description options("nc_contour_spiral");
    std::vector<std::string> args(argv, argv + argc);
    args.erase(begin(args));

    options.add(machine_config::base_options());
    options.add_options()
        ("help,h", "display this help and exit")
        ("tool_r,r", po::value<double>()->required(), "Tool radius")
        ("stepover,s", po::value<double>()->default_value(0.9), "Tool stepover 0.0 - 1.0")
        ("feedrate,f", po::value<double>()->required(), "Feedrate")
    ;

    try {
        po::variables_map vm;
        store(po::command_line_parser(args).options(options).run(), vm);

        if(vm.count("help")) {
            std::cout << options << "\n";
            return 0;
        }
        notify(vm);

        rs274_clipper_path nc_path(vm);
        double tool_offset = vm["tool_r"].as<double>() * 2 * vm["stepover"].as<double>();
        double feedrate = vm["feedrate"].as<double>();

        auto process_path = [&] (ClipperLib::Paths paths, double z) {

            // Offset path inwards by tool radius.
            {
                double offset = -vm["tool_r"].as<double>();
                double scale = nc_path.scale();

                cl::ClipperOffset co;
                co.AddPaths(paths, cl::jtRound, cl::etClosedPolygon);
                co.ArcTolerance = 0.025 * scale;
                co.Execute(paths, offset * scale);
            }

            Node root;
            root.polygon = paths;
            offset_node(root, -tool_offset, nc_path.scale());

            std::vector<Node> segments;
            segment(root, segments);

            for (auto& node : segments) {
                // TODO modify segments into spiral paths by interpolating between levels
                fprintf(stderr, "node.children.size(): %d\n", node.children.size());
                output_node(node, z, nc_path.scale(), 0);
            }

            std::cout << "\n";
        };

        // TODO read default init line from nc_tools.conf
//        nc_path.read("G18");
//        nc_path.execute();

        nc_path.set_callback(process_path);

        std::string line;
        while(std::getline(std::cin, line)) {
            int status;

            status = nc_path.read(line.c_str());
            if(status != RS274NGC_OK) {
                if(status != RS274NGC_EXECUTE_FINISH) {
                    std::cerr << "Error reading line!: \n";
                    std::cout << line <<"\n";
                    return status;
                }
            }
            
            status = nc_path.execute();
            if(status != RS274NGC_OK)
                return status;
        }

        if (nc_path.read("M2") == RS274NGC_OK)
            nc_path.execute();        

    } catch(const po::error& e) {
        print_exception(e);
        std::cout << options << "\n";
        return 1;
    } catch(const std::exception& e) {
        print_exception(e);
        return 1;
    }
}

