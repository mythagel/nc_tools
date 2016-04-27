#include "rs274ngc_return.hh"
#include <boost/program_options.hpp>
#include "print_exception.h"
#include "../throw_if.h"
#include "rs274_lathe_path.h"

#include <iostream>
#include <vector>
#include <vector>
#include <map>

#include "Bbox.h"
#include "Units.h"
#include <boost/math/special_functions/sign.hpp>

namespace po = boost::program_options;
namespace cl = ClipperLib;

cxxcam::Bbox bounding_box(const cl::Path& path, double scale) {
    cxxcam::Bbox bbox;
    if (path.empty())
        return bbox;

    auto pos2point = [&](const cl::IntPoint& p) {
        using namespace cxxcam::units;

        cxxcam::math::point_3 cp;
        cp.x = length{(p.X/scale) * millimeters};
        cp.z = length{(p.Y/scale) * millimeters};
        return cp;
    };

    auto p = begin(path);
    bbox.max = bbox.min = pos2point(*p);
    for (; p != end(path); ++p) {
        bbox += pos2point(*p);
    }
    return bbox;
}

void monotonic(const cl::Path& path, double scale, bool& monotonic_x, bool& monotonic_z) {
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

int main(int argc, char* argv[]) {
    po::options_description options("nc_lathe_roughing");
    std::vector<std::string> args(argv, argv + argc);
    args.erase(begin(args));

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

        rs274_path nc_path;

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

        auto start_x = nc_path.start_x();
        auto start_z = nc_path.start_z();
        auto path = nc_path.path();

        bool monotonic_x = false;
        bool monotonic_z = false;

        monotonic(path, nc_path.scale(), monotonic_x, monotonic_z);
        auto bbox = bounding_box(path, nc_path.scale());
        std::cerr << "monotonic x: " << monotonic_x << " monotonic z: " << monotonic_z << "\n";
        std::cerr << bbox << "\n";

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
        unsigned passes = std::abs(cxxcam::units::length_mm(bbox.max.x - bbox.min.x).value()) / vm["stepdown"].as<double>();
        double step_x = cxxcam::units::length_mm(bbox.max.x - bbox.min.x).value() / static_cast<double>(passes);

        auto equal = [](double a, double b, double tolerance = 1e-6) {
            return std::abs(b-a) < tolerance;
        };

        std::cerr << "start {" << start_x << ", " << start_z << "}\n";

        using namespace cxxcam::units;
        if (equal(start_x, length_mm(bbox.min.x).value(), 0.001)) {
            x = length_mm(bbox.min.x).value();
            x1 = length_mm(bbox.max.x).value();
        } else if (equal(start_x, length_mm(bbox.max.x).value())) {
            x = length_mm(bbox.max.x).value();
            x1 = length_mm(bbox.min.x).value();
            step_x = -step_x;
        } else {
            throw std::runtime_error("Initial X position must be located at min or max profile position");
        }

        if (equal(start_z, length_mm(bbox.min.z).value()), 0.001) {
            z0 = length_mm(bbox.min.z).value();
            z1 = length_mm(bbox.max.z).value();
        } else if (equal(start_z, length_mm(bbox.max.z).value())) {
            z0 = length_mm(bbox.max.z).value();
            z1 = length_mm(bbox.min.z).value();
        } else {
            throw std::runtime_error("Initial Z position must be located at min or max profile position");
        }

        // close path
        auto closed_path = path;
        closed_path.insert(closed_path.begin(), cl::IntPoint(cl::cInt(x1*nc_path.scale()), cl::cInt(z0*nc_path.scale())));
        closed_path.insert(closed_path.end(), cl::IntPoint(cl::cInt(x1*nc_path.scale()), cl::cInt(z1*nc_path.scale())));

        auto debug_path = [&](const cl::Path& path, bool close = true) {
            std::cout << std::fixed << "G00 X" << static_cast<double>(path.begin()->X)/nc_path.scale() << " Z" << static_cast<double>(path.begin()->Y)/nc_path.scale() << "\n";
            for(auto& p : path) {
                std::cout << std::fixed << "G01 X" << static_cast<double>(p.X)/nc_path.scale() << " Z" << static_cast<double>(p.Y)/nc_path.scale() << " F50\n";
            }
            if (close)
                std::cout << std::fixed << "G01 X" << static_cast<double>(path.begin()->X)/nc_path.scale() << " Z" << static_cast<double>(path.begin()->Y)/nc_path.scale() << "\n";
            std::cout << "\n";
            std::cout << "\n";
        };

        //debug_path(path);

        std::map<unsigned, cl::Paths> paths;
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
            clpr.AddPath(closed_path, cl::ptClip, true);
            cl::Paths solution;
            clpr.Execute(cl::ctDifference, solution, cl::pftEvenOdd, cl::pftEvenOdd);
            paths[pass] = solution;

            // something
            // create closed path from input path
            // then do difference

            x += step_x;
        }
        for (auto& pass : paths)
            for (auto& path : pass.second)
                debug_path(path);
        debug_path(path, false);

    } catch(const po::error& e) {
        print_exception(e);
        std::cout << options << "\n";
        return 1;
    } catch(const std::exception& e) {
        print_exception(e);
        return 1;
    }
}

void blah(const std::map<unsigned, cl::Paths>& paths, double scale, unsigned pass, double x0, double x1) {
    auto path = paths.find(pass);
    if (path == end(paths)) return;
    for (auto& cut : path->second) {

        /* for each depth, output this depth then recurse to lower depths (AT THE SPECIFIC X POSITION)
         * */
    }
}
