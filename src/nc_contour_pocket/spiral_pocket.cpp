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

int main(int argc, char* argv[]) {
    po::options_description options("nc_spiral_pocket");
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

        point_2 current_point = {0, 0};
        std::cout << "G0 Z" << r6(retract_z) << "\n";

        for (const auto& path : paths) {

            double z = step_z;
            for (unsigned step = 0; step < n_steps; ++step, z += step_z) {

                bool rapid_to_first = true;

                auto plunge = [&] {
                    // TODO helix to depth!!
                    // G2 x0 y0 I1.5 J1.5 Z-5 p10 f200
                    std::cout << "G1 Z" << r6(z) << " F" << r6(feedrate/2) << "\n";
                };
                auto scale_point = [&](const point_2& p) -> cl::IntPoint {
                    return cl::IntPoint(p.x * nc_path.scale(), p.y * nc_path.scale());
                };
                auto unscale_point = [&](const cl::IntPoint& p) -> point_2 {
                    return {static_cast<double>(p.X)/nc_path.scale(), static_cast<double>(p.Y)/nc_path.scale()};
                };

                // Find polygon centroid
                // determine incircle radius (approximate!)
                // helix down to depth 1.5x cutter radius
                // Create spiral, clip to path
                std::vector<point_2> scaled_path;
                std::transform(begin(path), end(path), std::back_inserter(scaled_path), [&](const cl::IntPoint& p) {
                    return unscale_point(p);
                });

                auto c = centroid(scaled_path);
                //std::cout << "G83 X" << r6(c.x) << " Y" << r6(c.y) << " Z-1 R1 Q0.5 F50" << '\n';

                double min_radius = 0.0;
                double max_radius = 0.0;
                {
                    auto it = std::minmax_element(begin(scaled_path), end(scaled_path), [&c](const point_2& p0, const point_2& p1) -> bool {
                        return distance(c, p0) < distance(c, p1);
                    });

                    min_radius = distance(c, *it.first);
                    max_radius = distance(c, *it.second);
                }

                // create spiral points, starting at c up to max_radius.
                cl::Path spiral_path;
                {
                    double turn_theta = 2*PI * (max_radius / tool_offset);
                    double rad_per_theta = max_radius / turn_theta;
                    for (double theta = 0.1; theta < turn_theta; theta += 0.1) {
                        double r = rad_per_theta * theta;
                        double x = c.x + std::cos(theta) * r;
                        double y = c.y + std::sin(theta) * r;
                        spiral_path.push_back(scale_point({x, y}));
                    }
                }
                cl::Clipper clipper;
                clipper.AddPath(spiral_path, cl::ptSubject, false);
                clipper.AddPath(path, cl::ptClip, true);

                cl::PolyTree pt;
                clipper.Execute(cl::ctIntersection, pt);

                // TODO sort subpaths!!
                for(auto node = pt.GetFirst(); node; node = node->GetNext()) {
                    auto& path = node->Contour;
                    if (rapid_to_first) {
                        auto p = unscale_point(path.front());
                        std::cout << "G0 X" << r6(p.x) << " Y" << r6(p.y) << "\n";
                        std::cout << "G1 Z" << r6(z) << " F" << r6(feedrate/2) << "\n";

                        rapid_to_first = false;
                    }

                    // Close path
                    //scaled_path.push_back(scaled_path.front());
                    for(auto& point : path) {
                        auto p = unscale_point(point);
                        std::cout << "   X" << r6(p.x) << " Y" << r6(p.y) << "\n";
                    }
                    std::cout << "\n";
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

