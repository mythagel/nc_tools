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


        auto orient_path = [&climb](cl::Path& path) {
            bool CCW = Orientation(path);
            if (climb && !CCW)
                ReversePath(path);
            else if (!climb && CCW)
                ReversePath(path);
        };

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

        point_2 current_point = {0, 0};
        std::cout << "G0 Z" << r6(retract_z) << "\n";

        for (const auto& path : paths) {

            /* TODO Offsetting paths individually makes improving the cut order easier,
             * but prevents the handling of pockets with islands.
             *
             * Use the PolyTree structure to handle offsetting of paths with islands, and maintain
             * the parent-child relationship needed to traverse z order first. */
            cl::ClipperOffset co;
            co.AddPath(path, cl::jtRound, cl::etClosedPolygon);
            co.ArcTolerance = 0.1 * nc_path.scale();

            double z = step_z;
            for (unsigned step = 0; step < n_steps; ++step, z += step_z) {

                bool rapid_to_first = true;

                double offset = 0.0;
                while (true) {
                    cl::Paths solution;
                    co.Execute(solution, offset * nc_path.scale());

                    auto unscale_point = [&](const cl::IntPoint& p) -> point_2 {
                        return {static_cast<double>(p.X)/nc_path.scale(), static_cast<double>(p.Y)/nc_path.scale()};
                    };

                    for(auto& path : solution) {

                        // Reorient path wrt. the cut direction (climb vs conventional)
                        orient_path(path);

                        auto it_near = std::min_element(begin(path), end(path), 
                            [&](const cl::IntPoint& l, const cl::IntPoint& r) -> bool {
                                return std::abs(distance(unscale_point(l), current_point)) < std::abs(distance(unscale_point(r), current_point));
                            });
                        std::rotate(begin(path), it_near, end(path));

                        if (rapid_to_first) {
                            auto p = unscale_point(path.front());
                            std::cout << "G0 X" << r6(p.x) << " Y" << r6(p.y) << "\n";
                            std::cout << "G1 Z" << r6(z) << " F" << r6(feedrate/2) << "\n";

                            rapid_to_first = false;
                        }

                        // Close path
                        path.push_back(path.front());
                        for(auto& point : path) {
                            auto p = unscale_point(point);
                            std::cout << "   X" << r6(p.x) << " Y" << r6(p.y) << "\n";
                        }
                        std::cout << "\n";

                        current_point = unscale_point(path.front());
                    }

                    if(solution.empty())
                        break;

                    offset -= tool_offset;
                }

                std::cout << "G0 Z" << r6(retract_z) << "\n";
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

