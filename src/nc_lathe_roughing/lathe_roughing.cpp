#include "rs274ngc_return.hh"
#include <boost/program_options.hpp>
#include "print_exception.h"
#include "../throw_if.h"
#include "rs274_lathe_path.h"
#include "base/machine_config.h"

#include <iostream>
#include <vector>
#include <vector>
#include <map>

#include "Units.h"
#include <boost/math/special_functions/sign.hpp>

namespace po = boost::program_options;
namespace cl = ClipperLib;

struct Bbox {
    cl::IntPoint min;
    cl::IntPoint max;
};
Bbox bounding_box(const cl::Path& path) {
    Bbox bbox;
    if (path.empty())
        return bbox;

    auto p = begin(path);
    bbox.max = bbox.min = *p;
    for (; p != end(path); ++p) {
        bbox.min.X = std::min(bbox.min.X, p->X);
        bbox.min.Y = std::min(bbox.min.Y, p->Y);
        bbox.max.X = std::max(bbox.max.X, p->X);
        bbox.max.Y = std::max(bbox.max.Y, p->Y);
    }
    return bbox;
}

void monotonic(const cl::Path& path, bool& monotonic_x, bool& monotonic_z) {
    int current_dir_x = 0;
    int current_dir_z = 0;
    monotonic_x = true;
    monotonic_z = true;
    if (path.empty())
        return;

    auto p = begin(path);
    auto a = *p++;
    for (; p != end(path); ++p) {
        auto b = *p;
        auto dir_x = boost::math::sign(b.X - a.X);
        auto dir_z = boost::math::sign(b.Y - a.Y);
        
        if (dir_x != 0) {
            if (current_dir_x == 0) current_dir_x = dir_x;
            if (dir_x != current_dir_x) monotonic_x = false;
        }
        if (dir_z != 0) {
            if (current_dir_z == 0) current_dir_z = dir_z;
            if (dir_z != current_dir_z) monotonic_z = false;
        }
    }
}

cl::Paths overlap_z(const cl::Path& subject, const cl::Paths& candidates) {
    cl::Paths overlap;
    auto bbox = bounding_box(subject);
    for(auto& path : candidates) {
        auto b = bounding_box(path);
        if (b.min.Y > bbox.max.Y)
            continue;
        if (b.max.Y < bbox.min.Y)
            continue;
        overlap.push_back(path);
    }
    return overlap;
}
void output_path(const cl::Path& path, double scale, bool close = true) {
    std::cout << std::fixed << "G00 X" << static_cast<double>(path.begin()->X)/scale << " Z" << static_cast<double>(path.begin()->Y)/scale << "\n";
    for(auto& p : path) {
        std::cout << std::fixed << "G01 X" << static_cast<double>(p.X)/scale << " Z" << static_cast<double>(p.Y)/scale << " F200\n";
    }
    if (close)
        std::cout << std::fixed << "G01 X" << static_cast<double>(path.begin()->X)/scale << " Z" << static_cast<double>(path.begin()->Y)/scale << "\n";
    std::cout << "\n";
}
void blah(const std::vector<cl::Paths>& levels, double scale, const cl::Path* subject = NULL, unsigned x_level = 0) {
    if (levels.empty()) return;

    if (!subject) {
        // first level
        for (auto& path : levels[x_level])
            blah(levels, scale, &path, x_level);

    } else {
        output_path(*subject, scale);
        if(x_level+1 < levels.size()) {
            auto& next_level = levels[x_level+1];
            auto overlap_next = overlap_z(*subject, next_level);

            for (auto& path : overlap_next) {
                blah(levels, scale, &path, x_level+1);
            }
        }
    }
}

int main(int argc, char* argv[]) {
    po::options_description options("nc_lathe_roughing");
    std::vector<std::string> args(argv, argv + argc);
    args.erase(begin(args));

    options.add(machine_config::base_options());
    options.add_options()
        ("help,h", "display this help and exit")
        ("stepdown,D", po::value<double>()->required(), "roughing stepdown")
        ("retract,R", po::value<double>()->default_value(0.5), "retraction per cut")
    ;

    try {
        po::variables_map vm;
        store(po::command_line_parser(args).options(options).run(), vm);

        if(vm.count("help")) {
            std::cout << options << "\n";
            return 0;
        }
        notify(vm);

        rs274_path nc_path(vm);

        nc_path.read("G18");
        nc_path.execute();

        std::string line;
        while(std::getline(std::cin, line)) {
            int status;

            status = nc_path.read(line.c_str());
            if(status != RS274NGC_OK) {
                if(status != RS274NGC_EXECUTE_FINISH) {
                    std::cerr << "Error reading line!: \n";
                    std::cerr << line <<"\n";
                    return status;
                }
            }
            
            status = nc_path.execute();
            if(status != RS274NGC_OK)
                return status;
        }

        auto start = nc_path.start();
        auto path = nc_path.path();

        auto unscale = [&](cl::cInt x) {
            return static_cast<double>(x) / nc_path.scale();
        };

        bool monotonic_x = false;
        bool monotonic_z = false;

        monotonic(path, monotonic_x, monotonic_z);
        auto bbox = bounding_box(path);
        std::cerr << "monotonic x: " << monotonic_x << " monotonic z: " << monotonic_z << "\n";
        std::cerr << "min: {" << unscale(bbox.min.X) << "," << unscale(bbox.min.Y) << "} max: {" << unscale(bbox.max.X) << "," << unscale(bbox.max.Y) << "}\n";

        /* Find which corner of path tool is positioned at, infer whether
         * motion is in +x or -x and +z or -z
         *
         * calculate number of passes, abs x depth <integer divide> depth of cut gives number of passes
         * abs x depth / number of passes gives exact depth per pass
         */
        double x;
        double x1;
        double z0;
        double z1;

        auto equal = [](double a, double b, double tolerance = 1e-6) {
            return std::abs(b-a) < tolerance;
        };

        std::cerr << "start {" << unscale(start.X) << ", " << unscale(start.Y) << "}\n";

        using namespace cxxcam::units;
        if (equal(unscale(start.X), unscale(bbox.min.X), 1e-3)) {
            x = unscale(bbox.max.X);
            x1 = unscale(bbox.min.X);
        } else if (equal(unscale(start.X), unscale(bbox.max.X)), 1e-3) {
            x = unscale(bbox.min.X);
            x1 = unscale(bbox.max.X);
        } else {
            throw std::runtime_error("Initial X position must be located at min or max profile position");
        }

        if (equal(unscale(start.Y), unscale(bbox.min.Y)), 1e-3) {
            z0 = unscale(bbox.max.Y);
            z1 = unscale(bbox.min.Y);
        } else if (equal(start.Y, unscale(bbox.max.Y)), 1e-3) {
            z0 = unscale(bbox.min.Y);
            z1 = unscale(bbox.max.Y);
        } else {
            throw std::runtime_error("Initial Z position must be located at min or max profile position");
        }

        unsigned passes = std::abs(unscale(bbox.max.X - bbox.min.X)) / vm["stepdown"].as<double>();
        double step_x = (x1 - x) / static_cast<double>(passes);

        // close path
        cl::Paths closed_paths;
        auto close_path = [&](cl::Path path) {
            auto contains = [&path](const cl::IntPoint& p) {
                return std::find(begin(path), end(path), p) != end(path);
            };

            auto p0 = cl::IntPoint(cl::cInt(x1*nc_path.scale()), cl::cInt(z0*nc_path.scale()));
            auto p1 = cl::IntPoint(cl::cInt(x1*nc_path.scale()), cl::cInt(z1*nc_path.scale()));
            if (!contains(p0))
                path.insert(path.begin(), p0);
            if (!contains(p1))
                path.insert(path.end(), p1);
            return path;
        };

        closed_paths.push_back(close_path(path));

        auto debug_path = [&](const cl::Path& path, bool close = true) {
            if (path.empty()) return;
            std::cout << std::fixed << "G00 X" << static_cast<double>(path.begin()->X)/nc_path.scale() << " Z" << static_cast<double>(path.begin()->Y)/nc_path.scale() << "\n";
            for(auto& p : path) {
                std::cout << std::fixed << "G01 X" << static_cast<double>(p.X)/nc_path.scale() << " Z" << static_cast<double>(p.Y)/nc_path.scale() << " F200\n";
            }
            if (close)
                std::cout << std::fixed << "G01 X" << static_cast<double>(path.begin()->X)/nc_path.scale() << " Z" << static_cast<double>(path.begin()->Y)/nc_path.scale() << "\n";
            std::cout << "\n";
            std::cout << "\n";
        };

        if (true)
        {
            cl::ClipperOffset co;
            co.AddPath(closed_paths[0], cl::jtRound, cl::etClosedPolygon);
            co.ArcTolerance = 0.1 * nc_path.scale();

            cl::Paths solution;
            co.Execute(solution, -0.1 * nc_path.scale());
            std::swap(closed_paths, solution);
        }

        std::vector<cl::Paths> paths;
        for (unsigned pass = 0; pass < passes; ++pass) {
            auto sclp = [&](double x, double y) {
                return cl::IntPoint { cl::cInt(x*nc_path.scale()), cl::cInt(y*nc_path.scale()) };
            };
            cl::Path step;
            step.push_back(sclp(x, z0));
            step.push_back(sclp(x, z1));
            step.push_back(sclp(x+step_x, z1));
            step.push_back(sclp(x+step_x, z0));

            //debug_path(step);

            cl::Clipper clpr;
            clpr.AddPath(step, cl::ptSubject, true);
            for (auto& path : closed_paths)
                clpr.AddPath(path, cl::ptClip, true);
            cl::Paths solution;
            clpr.Execute(cl::ctIntersection, solution, cl::pftEvenOdd, cl::pftEvenOdd);
            paths.insert(paths.begin(), solution);

            //for (auto& path : solution)
            //    debug_path(path);

            // something
            // create closed path from input path
            // then do difference

            x += step_x;
        }
        blah(paths, nc_path.scale());
        output_path(path, nc_path.scale(), false);
/*        for (auto& pass : paths)
            for (auto& path : pass)
                debug_path(path);
        debug_path(path, false);
*/
    } catch(const po::error& e) {
        print_exception(e);
        std::cout << options << "\n";
        return 1;
    } catch(const std::exception& e) {
        print_exception(e);
        return 1;
    }
}

