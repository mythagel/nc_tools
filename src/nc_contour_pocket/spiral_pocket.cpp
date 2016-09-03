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
#include <stdexcept>

namespace po = boost::program_options;
namespace cl = ClipperLib;
using namespace geometry;

std::vector<std::vector<point_2>> spiral_zigzag(const std::vector<point_2>& path, double tool_offset) {
    auto c = centroid(path);

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

    double min_radius = 0.0;
    double max_radius = 0.0;
    {
        auto it = std::minmax_element(begin(path), end(path), 
            [&c](const point_2& p0, const point_2& p1) -> bool {
                return distance(c, p0) < distance(c, p1);
            });

        min_radius = distance(c, *it.first);
        max_radius = distance(c, *it.second);
    }
    // TODO check min radius to determine if reasonable to helix into pocket for initial plunge.

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

    cl::Paths paths;
    {
        cl::Clipper clipper;
        clipper.AddPath(spiral_path, cl::ptSubject, false);
        clipper.AddPath(scale_path(path), cl::ptClip, true);

        cl::PolyTree pt;
        clipper.Execute(cl::ctIntersection, pt);
        OpenPathsFromPolyTree(pt, paths);
    }

    std::vector<std::vector<point_2>> toolpaths;

    // Start at center point of spiral
    point_2 current_point = c;

    bool new_path = true;
    while (!paths.empty()) {

        auto it_min_start = std::min_element(begin(paths), end(paths), 
            [&](const cl::Path& p0, const cl::Path& p1) -> bool {
                return distance(unscale_point(p0.front()), current_point) < distance(unscale_point(p1.front()), current_point);
            });
        auto it_min_end = std::min_element(begin(paths), end(paths), 
            [&](const cl::Path& p0, const cl::Path& p1) -> bool {
                return distance(unscale_point(p0.back()), current_point) < distance(unscale_point(p1.back()), current_point);
            });

        auto dist_start = distance(unscale_point(it_min_start->front()), current_point);
        auto dist_end = distance(unscale_point(it_min_end->back()), current_point);

        auto it = dist_start < dist_end ? it_min_start : it_min_end;
        auto path = *it;
        paths.erase(it);

        double dist;
        if (dist_end < dist_start) {
            ReversePath(path);
            dist = dist_end;
        } else {
            dist = dist_start;
        }

        // TODO how to better characterise this constant?
        // Represents the threshold of distance between two paths
        // which would result in a retract
        if (dist >= tool_offset*3) {
            new_path = true;
        }

        // next path closest to end point of this path.
        current_point = unscale_point(path.back());

        if (new_path) {
            toolpaths.emplace_back();
            new_path = false;
        }

        for(auto& point : path) {
            auto p = unscale_point(point);
            toolpaths.back().push_back(p);
        }
    }

    return toolpaths;
}

std::vector<std::vector<point_2>> spiral_morph(const std::vector<point_2>& path, double tool_offset) {
	auto c = centroid(path);

	double min_radius = 0.0;
	double max_radius = 0.0;
	{
		auto it = std::minmax_element(begin(path), end(path), [&c](const point_2& p0, const point_2& p1) -> bool {
			return distance(c, p0) < distance(c, p1);
		});

		min_radius = distance(c, *it.first);
		max_radius = distance(c, *it.second);
	}

    std::vector<std::vector<point_2>> toolpaths;
    toolpaths.emplace_back();

	double radius = min_radius;
	double coil_gap = 0.3;
	double turn_theta = 2*PI * (radius / coil_gap);
	double rad_per_theta = radius / turn_theta;
	for (double theta = 0.1; theta < turn_theta; theta += 0.03) {
		double r = rad_per_theta * theta;
		point_2 p0 = {c.x + std::cos(theta) * r, c.y + std::sin(theta) * r};
		line_segment_2 ray;
		ray.a = c;
		ray.b = {c.x + std::cos(theta) * (max_radius+1), c.y + std::sin(theta) * (max_radius+1)}; // +1 == fudge
		auto p1 = intersects(ray, path);
		if (!p1) throw std::runtime_error("path must be closed");
		auto t = theta / turn_theta;
		auto p = lerp(p0, *p1, t);

		toolpaths.back().push_back(p);
	}
    return toolpaths;
}

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

            const auto paths = spiral_zigzag(scaled_path, tool_offset);

            // Sprial morph not ready.
            //auto paths = spiral_morph(scaled_path, tool_offset);

            double z = step_z;
            for (unsigned step = 0; step < n_steps; ++step, z += step_z) {

                bool helical_plunge = true;
                for (auto& path : paths) {
                    bool rapid_to_first = true;

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

                    std::cout << "G0 Z" << r6(retract_z) << "\n";
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

