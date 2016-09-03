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

std::vector<std::vector<point_2>> contour_offset(const std::vector<point_2>& path, double tool_offset, bool climb) {
    double scale = 10e12;
    auto scale_point = [&](const point_2& p) -> cl::IntPoint {
        return cl::IntPoint(p.x * scale, p.y * scale);
    };
    auto unscale_point = [&](const cl::IntPoint& p) -> point_2 {
        return {static_cast<double>(p.X)/scale, static_cast<double>(p.Y)/scale};
    };
    auto scale_path = [&](const std::vector<point_2>& path) {
        cl::Path scaled;
        scaled.reserve(path.size());
        for (auto& p : path)
            scaled.push_back(scale_point(p));
        return scaled;
    };

    cl::ClipperOffset co;
    co.AddPath(scale_path(path), cl::jtRound, cl::etClosedPolygon);
    co.ArcTolerance = 0.1 * scale;

    std::vector<std::vector<std::vector<point_2>>> toolpaths_levels;
    point_2 current_point = {0, 0};

    double offset = 0.0;
    while (true) {

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
    po::options_description options("nc_contour_pocket");
    std::vector<std::string> args(argv, argv + argc);
    args.erase(begin(args));

    options.add(machine_config::base_options());
    options.add_options()
        ("help,h", "display this help and exit")
        ("tool_r,r", po::value<double>()->required(), "Tool radius")
        ("stepover,s", po::value<double>()->default_value(0.9), "Tool stepover 0.0 - 1.0")
        ("cut_z,z", po::value<double>()->required(), "Z Cut depth")
        ("feedrate,f", po::value<double>()->required(), "Z Cut depth")
        ("stepdown,d", po::value<double>()->required(), "Z Stepdown")
        ("retract_z,t", po::value<double>()->default_value(1.0), "Z Tool retract")
        ("climb,c", "Climb mill (Relative to clockwise cutter rotation)")
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
        double cut_z = vm["cut_z"].as<double>();
        double feedrate = vm["feedrate"].as<double>();
        double stepdown = vm["stepdown"].as<double>();
        double retract_z = vm["retract_z"].as<double>();
        bool climb = vm.count("climb");

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

        auto paths = nc_path.path();

        const unsigned n_steps = std::abs(std::ceil(cut_z / stepdown));
        const double step_z = cut_z / n_steps;

        std::cout << "G0 Z" << r6(retract_z) << "\n";

        for (const auto& path : paths) {

            /* TODO Offsetting paths individually makes improving the cut order easier,
             * but prevents the handling of pockets with islands.
             *
             * Use the PolyTree structure to handle offsetting of paths with islands, and maintain
             * the parent-child relationship needed to traverse z order first. */

            auto unscale_point = [&](const cl::IntPoint& p) -> point_2 {
                return {static_cast<double>(p.X)/nc_path.scale(), static_cast<double>(p.Y)/nc_path.scale()};
            };

            const auto scaled_path = [&] {
                std::vector<point_2> scaled;
                scaled.reserve(path.size());
                std::transform(begin(path), end(path), std::back_inserter(scaled), [&](const cl::IntPoint& p) {
                    return unscale_point(p);
                });
                return scaled;
            }();

            const auto paths = contour_offset(scaled_path, tool_offset, climb);

            double z = step_z;
            for (unsigned step = 0; step < n_steps; ++step, z += step_z) {

                bool rapid_to_first = true;
                bool helical_plunge = true;
                for (auto& path : paths) {

                    if (rapid_to_first) {
                        auto p = path.front();
                        std::cout << "G0 X" << r6(p.x) << " Y" << r6(p.y) << "\n";

                        if (helical_plunge) {
                            double plunge_r = vm["tool_r"].as<double>();
                            unsigned plunge_p = std::abs(z) / 0.1;   // TODO plunge stepdown
                            std::cout << "G0 X" << r6(p.x - plunge_r) << " Y" << r6(p.y) << "\n";
                            std::cout << "G3 X" << r6(p.x - plunge_r) << " Y" << r6(p.y) 
                                        << " I" << r6(plunge_r) 
                                        << " P" << plunge_p 
                                        << " Z" << z 
                                        << " F" << r6(feedrate) << "\n";
                            std::cout << "G1 X" << r6(p.x) << " Y" << r6(p.y) << "\n";

                            helical_plunge = false;
                        } else {
                            std::cout << "G0 X" << r6(p.x) << " Y" << r6(p.y) << "\n";
                            std::cout << "G1 Z" << r6(z) << " F" << r6(feedrate/2) << "\n";
                        }
                        rapid_to_first = false;
                    }

                    for(auto& p : path) {
                        std::cout << "   X" << r6(p.x) << " Y" << r6(p.y) << "\n";
                    }
                }
                std::cout << "G0 Z" << r6(retract_z) << "\n";
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

