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
#include "cxxcam/Path.h"

namespace po = boost::program_options;
namespace cl = ClipperLib;
using namespace geometry;

int main(int argc, char* argv[]) {
    po::options_description options("nc_contour_profile");
    std::vector<std::string> args(argv, argv + argc);
    args.erase(begin(args));

    options.add(machine_config::base_options());
    options.add_options()
        ("help,h", "display this help and exit")
        ("tool_r,r", po::value<double>()->required(), "Tool radius")
        ("cut_z,z", po::value<double>()->required(), "Z Cut depth")
        ("feedrate,f", po::value<double>()->required(), "Z Cut depth")
        ("stepdown,d", po::value<double>()->required(), "Z Stepdown")
        ("retract_z,t", po::value<double>()->default_value(1.0), "Z Tool retract")
        ("climb,c", "Climb mill (Relative to clockwise cutter rotation)")
        ("spiral", "Spiral stepdown")
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
        double cut_z = vm["cut_z"].as<double>();
        double feedrate = vm["feedrate"].as<double>();
        double stepdown = vm["stepdown"].as<double>();
        double retract_z = vm["retract_z"].as<double>();
        bool climb = vm.count("climb");
        bool spiral = vm.count("spiral");

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

        auto paths = nc_path.path();

        // Offset path outwards by tool radius.
        {
            cl::ClipperOffset co;
            co.AddPaths(paths, cl::jtRound, cl::etClosedPolygon);
            co.ArcTolerance = 0.1 * nc_path.scale();
            co.Execute(paths, vm["tool_r"].as<double>() * nc_path.scale());
        }
        auto orient_path = [&climb](cl::Path& path) {
            bool CCW = Orientation(path);
            if (climb && !CCW)
                ReversePath(path);
            else if (!climb && CCW)
                ReversePath(path);
        };

        for (auto& path : paths)
            orient_path(path);

        std::cout << "G0 Z" << r6(retract_z) << "\n";

        if (spiral == false) {
            const unsigned n_steps = std::abs(std::ceil(cut_z / stepdown));
            const double step_z = cut_z / n_steps;

            double z = step_z;
            for (unsigned step = 0; step < n_steps; ++step, z += step_z) {

                bool rapid_to_first = true;
                for (auto& path : paths) {

                    if (rapid_to_first) {
                        auto p = unscale_point(path.front());
                        std::cout << "G0 X" << r6(p.x) << " Y" << r6(p.y) << "\n";
                        std::cout << "G1 Z" << r6(z) << " F" << r6(feedrate/2) << "\n";
                        rapid_to_first = false;
                    }

                    // close path
                    path.push_back(path.front());
                    for(auto& point : path) {
                        auto p = unscale_point(point);
                        std::cout << "   X" << r6(p.x) << " Y" << r6(p.y) << "\n";
                    }
                }
                std::cout << "G0 Z" << r6(retract_z) << "\n";
                std::cout << "\n";
            }
        } else {
            for (auto& path : paths) {

                // close path
                path.push_back(path.front());
                bool rapid_to_first = true;

                double L = 0;
                {
                    auto prev = unscale_point(path.front());
                    for(auto& point : path) {
                        auto p = unscale_point(point);
                        L += distance(prev, p);
                        prev = p;
                    }
                }

                double z = 0;
                while (z > cut_z) {
                    if (rapid_to_first) {
                        auto p = unscale_point(path.front());
                        std::cout << "G0 X" << r6(p.x) << " Y" << r6(p.y) << "\n";
                        std::cout << "G1 Z" << r6(z) << " F" << r6(feedrate/2) << "\n";
                        rapid_to_first = false;
                    }

                    auto prev = unscale_point(path.front());
                    int last_loop_end = -1;
                    for(unsigned i = 0; i < path.size(); ++i) {
                        auto p = unscale_point(path[i]);

                        if (last_loop_end == -1) {
                            auto l = distance(prev, p);
                            z += (stepdown / L) * l;
                            if (z <= cut_z) {
                                z = cut_z;
                                last_loop_end = i;
                            }
                            prev = p;
                        }

                        std::cout << "   X" << r6(p.x) << " Y" << r6(p.y) << " Z" << r6(z) << "\n";
                    }

                    if (last_loop_end > 0) {
                        for(int i = 0; i < last_loop_end; ++i) {
                            auto p = unscale_point(path[i]);
                            std::cout << "   X" << r6(p.x) << " Y" << r6(p.y) << " Z" << r6(z) << "\n";
                        }
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

