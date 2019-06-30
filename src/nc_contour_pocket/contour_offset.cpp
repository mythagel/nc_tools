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

std::vector<std::vector<point_2>> contour_offset(const ClipperLib::Paths& paths, double tool_offset, bool climb) {
    double scale = 10e12;

    auto unscale_point = [&](const cl::IntPoint& p) -> point_2 {
        return {static_cast<double>(p.X)/scale, static_cast<double>(p.Y)/scale};
    };

    cl::ClipperOffset co;
    co.AddPaths(paths, cl::jtRound, cl::etClosedPolygon);
    co.ArcTolerance = 0.1 * scale;

    std::vector<std::vector<std::vector<point_2>>> toolpaths_levels;
    point_2 current_point = {0, 0};

    double offset = 0.0;
    while (offset <= 0) {

        toolpaths_levels.emplace_back();
        auto& toolpaths = toolpaths_levels.back();
        bool new_path = true;

        cl::Paths solution;
        co.Execute(solution, offset * scale);

        if(solution.empty())
            break;

        for(auto& path : solution) {

            auto orient_path = [&climb](cl::Path& path) {
                bool CCW = Orientation(path);
                if (climb && !CCW)
                    ReversePath(path);
                else if (!climb && CCW)
                    ReversePath(path);
            };

            orient_path(path);

            auto it_near = std::min_element(begin(path), end(path), 
                [&](const cl::IntPoint& l, const cl::IntPoint& r) -> bool {
                    return std::abs(distance(unscale_point(l), current_point)) < std::abs(distance(unscale_point(r), current_point));
                });
            double dist = distance(unscale_point(*it_near), current_point);
            std::rotate(begin(path), it_near, end(path));

            // TODO how to better characterise this constant?
            // Represents the threshold of distance between two paths
            // which would result in a retract
            if (dist >= tool_offset*3) {
                new_path = true;
            }

            // next path closes to start point of this path
            current_point = unscale_point(path.front());

            if (new_path) {
                toolpaths.emplace_back();
                new_path = false;
            }

            auto& toolpath = toolpaths.back();

            // cannot be reversed this way because path is continuous...
            for(auto& point : path) {
                auto p = unscale_point(point);
                toolpath.push_back(p);
            }
            // Close path
            toolpath.push_back(unscale_point(path.front()));
        }

        offset -= tool_offset;
    }

    std::reverse(begin(toolpaths_levels), end(toolpaths_levels));
    std::vector<std::vector<point_2>> toolpaths;
    for (auto& level : toolpaths_levels)
        toolpaths.insert(toolpaths.end(), begin(level), end(level));

    return toolpaths;
}

int main(int argc, char* argv[]) {
    po::options_description options("nc_contour_offset");
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
                double scale = 10e12;

                cl::ClipperOffset co;
                co.AddPaths(paths, cl::jtRound, cl::etClosedPolygon);
                co.ArcTolerance = 0.1 * scale;
                co.Execute(paths, offset * scale);
            }

            const auto offset_paths = contour_offset(paths, tool_offset, false);

            for (auto& path : offset_paths) {

                bool rapid_to_first = true;
                if (rapid_to_first) {
                    auto p = path.front();
                    std::cout << "G0 X" << r6(p.x) << " Y" << r6(p.y) << "\n";
                    std::cout << "G1 Z" << r6(z) << " F" << r6(feedrate/2) << "\n";
                    rapid_to_first = false;
                }

                for(auto& p : path) {
                    std::cout << "   X" << r6(p.x) << " Y" << r6(p.y) << " Z" << r6(z) << "\n";
                }
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

