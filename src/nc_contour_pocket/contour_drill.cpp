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

void fold_path (const cl::PolyNode* node, cl::Paths& paths) {
    paths.push_back(node->Contour);
    for (auto& child : node->Childs)
        fold_path(child, paths);
}

std::vector<cl::Paths> segment_paths(cl::Paths paths) {
    cl::Clipper clipper;
    clipper.AddPaths(paths, cl::ptSubject, true);

    cl::PolyTree pt;
    clipper.Execute(cl::ctUnion, pt);

    std::vector<cl::Paths> segments;

    segments.reserve(pt.Childs.size());
    for (auto& child : pt.Childs) {
        segments.emplace_back();
        fold_path(child, segments.back());
    }

    return segments;
}

cl::Paths generate_circular_drill_pattern(double radius, double drill_d, double offset, point_2 centroid, double scale) {
    auto scale_point = [&](const point_2& p) -> cl::IntPoint {
        return cl::IntPoint(p.x * scale, p.y * scale);
    };

    cl::Paths paths;
    paths.emplace_back();

    double inscribed_radius = 0;
    double inscribed_offset = drill_d * offset;
    while (inscribed_radius < radius) {

        unsigned n_holes = std::floor((2*PI * inscribed_radius) / inscribed_offset);
        auto theta = 2*PI / n_holes;
        double t = 0;

        if (n_holes == 0) {
            paths.back().push_back(scale_point({centroid.x, centroid.y}));
        }
        for (unsigned i = 0; i < n_holes; ++i, t += theta) {
            auto x = inscribed_radius * std::cos(t);
            auto y = inscribed_radius * std::sin(t);
            paths.back().push_back(scale_point({x + centroid.x, y + centroid.y}));
        }
        inscribed_radius += inscribed_offset;
    }

    return paths;
}

cl::Paths generate_offset_drill_pattern(double radius, double drill_d, double offset, point_2 centroid, double scale) {
    auto scale_point = [&](const point_2& p) -> cl::IntPoint {
        return cl::IntPoint(p.x * scale, p.y * scale);
    };

    cl::Paths paths;
    paths.emplace_back();

    double inscribed_offset = drill_d * offset;
    for (int y = -radius; y < radius; ++y) {
        for (int x = -radius; x < radius; ++x) {
            double y_off = (x % 2) == 0 ? inscribed_offset/2 : 0;
            paths.back().push_back(scale_point({(x*inscribed_offset) + centroid.x, (y*inscribed_offset) + y_off + centroid.y}));
        }
    }

    return paths;
}

int main(int argc, char* argv[]) {
    po::options_description options("nc_contour_drill");
    std::vector<std::string> args(argv, argv + argc);
    args.erase(begin(args));

    options.add(machine_config::base_options());
    options.add_options()
        ("help,h", "display this help and exit")
        ("drill_d,d", po::value<double>()->required(), "Drill diameter")
        ("offset,o", po::value<double>()->default_value(2.0), "Drill diameter offset")
        ("drill_z,z", po::value<double>()->required(), "Z drill depth")
        ("feedrate,f", po::value<double>()->required(), "Feedrate")
        ("retract_z,t", po::value<double>()->required(), "Z Tool retract height")
        ("circular,c", "Circular drill pattern")
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
        double drill_d = vm["drill_d"].as<double>();
        double offset = vm["offset"].as<double>();
        double drill_z = vm["drill_z"].as<double>();
        double feedrate = vm["feedrate"].as<double>();
        double retract_z = vm["retract_z"].as<double>();

        // TODO read default init line from nc_tools.conf
//        nc_path.read("G18");
//        nc_path.execute();

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

        auto unscale_point = [&](const cl::IntPoint& p) -> geometry::point_2 {
            return {static_cast<double>(p.X) / nc_path.scale(), static_cast<double>(p.Y) / nc_path.scale()};
        };

        auto segments = segment_paths(nc_path.path());
        for (auto& segment : segments) {

            std::vector<point_2> path;
            for (auto& point : segment[0]) {
                path.push_back(unscale_point(point));
            }

            auto centroid = geometry::centroid(path);

            double radius = 0;
            for (auto& point : path) {
                double dist = geometry::distance(centroid, point);
                if (dist > radius)
                    radius = dist;
            }

            // Offset segment inwards by tool radius.
            {
                double offset = -drill_d/2;
                double scale = nc_path.scale();

                cl::ClipperOffset co;
                co.AddPaths(segment, cl::jtRound, cl::etClosedPolygon);
                co.Execute(segment, offset * scale);
            }

            cl::PolyTree pt;
            cl::Paths paths;

            auto original_point = [&](const cl::IntPoint& p) -> bool {
                for (auto& path : paths)
                    if (std::find(begin(path), end(path), p) != end(path))
                        return true;
                return false;
            };

            if(vm.count("circular"))
                paths = generate_circular_drill_pattern(radius, drill_d, offset, centroid, nc_path.scale());
            else
                paths = generate_offset_drill_pattern(radius, drill_d, offset, centroid, nc_path.scale());

            cl::Clipper clipper;
            clipper.AddPaths(paths, cl::ptSubject, false);
            clipper.AddPaths(segment, cl::ptClip, true);
            clipper.Execute(cl::ctIntersection, pt);

            // Lazy scan as the number of drill locations is likely to be small
            unsigned num_holes = 0;
            for(auto node = pt.GetFirst(); node; node = node->GetNext()) {
                auto& path = node->Contour;
                for(auto& point : path) {
                    if (!original_point(point)) continue;
                    ++num_holes;
                }
            }

            double hole_area = PI*((drill_d/2) * (drill_d/2));
            double open_area = hole_area * num_holes;
            double closed_area = geometry::area(path);
            std::cout << "( open: " << (open_area / closed_area) * 100.0 << "% )\n";

            for(auto node = pt.GetFirst(); node; node = node->GetNext()) {
                auto& path = node->Contour;
                for(auto& point : path) {
                    if (!original_point(point)) continue;
                    auto p = unscale_point(point);
                    std::cout << "G83 X" << r6(p.x) << " Y" << r6(p.y) << " Z" << r6(drill_z) << "  R" << r6(retract_z) << " Q" << r6(drill_d) << " F" << r6(feedrate) << '\n';
                }
                std::cout << "\n";
            }
        }

    } catch(const po::error& e) {
        print_exception(e);
        std::cout << options << "\n";
        return 1;
    } catch(const std::exception& e) {
        print_exception(e);
        return 1;
    }
}

