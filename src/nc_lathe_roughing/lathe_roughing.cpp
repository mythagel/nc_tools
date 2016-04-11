#include "rs274ngc_return.hh"
#include <boost/program_options.hpp>
#include "print_exception.h"
#include "../throw_if.h"
#include "rs274_lathe_path.h"

#include <iostream>
#include <vector>
#include <string>
#include "geometry.h"

#include "Bbox.h"
#include "Units.h"
#include <boost/math/special_functions/sign.hpp>

namespace po = boost::program_options;

std::vector<unsigned> intersects(const std::vector<line_segment_2>& path, const line_segment_2& l2) {
    std::vector<unsigned> intersections;
    for (unsigned idx = 0; idx < path.size(); ++idx) {
        if (intersects(path[idx], l2)) {
            intersections.push_back(idx);
        }
    }
    return intersections;
}

cxxcam::Bbox bounding_box(const std::vector<line_segment_2>& path) {
    cxxcam::Bbox bbox;
    if (path.empty())
        return bbox;

    auto pos2point = [](const point_2& p) {
        using namespace cxxcam::units;

        cxxcam::math::point_3 cp;
        cp.x = length{p.x * millimeters};
        cp.z = length{p.z * millimeters};
        return cp;
    };

    auto line = begin(path);
    bbox.max = bbox.min = pos2point(line->a);
    for (; line != end(path); ++line) {
        bbox += pos2point(line->a);
        bbox += pos2point(line->b);
    }
    return bbox;
}

void monotonic(const std::vector<line_segment_2>& path, bool& monotonic_x, bool& monotonic_z) {
    int current_dir_x = 0;
    int current_dir_z = 0;
    monotonic_x = true;
    monotonic_z = true;

    for (auto& line : path) {
        auto dir_x = boost::math::sign(line.b.x - line.a.x);
        auto dir_z = boost::math::sign(line.b.z - line.a.z);
        
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

int traverse_z(const std::vector<line_segment_2>& path, double x, double step_x, unsigned pass, unsigned passes, double z0, double z1) {
    std::cerr << "x: " << x << "\n";
    // output line from z0 to z1, where z1 = ??
    // z0 = ??
    line_segment_2 sweep = {{x, z0}, {x, z1}};

    sweep.a.x = sweep.b.x = x;
    auto intersections = intersects(path, sweep);
    if (intersections.empty()) {
        std::cerr << "cut from z0: " << z0 << " to z1: " << z1 << " at x: " << x << "\n";
        std::cout << "G0 Z" << z0 << "\n";
        std::cout << "G1 X" << x << " f50\n";
        std::cout << "G1 Z" << z1 << " f50\n";

        if (pass+1 < passes)
            return traverse_z(path, x + step_x, step_x, pass+1, passes, z0, z1);
    } else {
        for (auto idx : intersections) {
            auto line = path[idx];
            auto intersection = intersects(line, sweep);
            if (!intersection) throw std::logic_error("Line segment does not intersect");

            std::cerr << "cut from z0: " << z0 << " to z1: " << intersection->z << " at x: " << x << "\n";
            std::cout << "G0 Z" << z0 << "\n";
            std::cout << "G1 X" << x << " f50\n";
            std::cout << "G1 Z" << intersection->z << " f50\n";
            traverse_z(path, x + step_x, step_x, pass+1, passes, z0, intersection->z);
        }
    }
    return pass;
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

        auto start_point = nc_path.start_point();
        auto path = nc_path.path();

        bool monotonic_x = false;
        bool monotonic_z = false;

        monotonic(path, monotonic_x, monotonic_z);
        auto bbox = bounding_box(path);
        std::cerr << "monotonic x: " << monotonic_x << " monotonic z: " << monotonic_z << "\n";
        std::cerr << bbox << "\n";


        // determine starting point and direction based on initial position
        /* Find which corner of path tool is positioned at, infer whether
         * motion is in +x or -x and +z or -z
         *
         * calculate number of passes, abs x depth <integer divide> depth of cut gives number of passes
         * abs x depth / number of passes gives exact depth per pass
         * cast ray at x depth, incremented by depthperpass
         * if no intersections, move from minz to maxz
         * otherwise, feed to first intersection, follow path until at x depth again, then feed to next intersection
         * */
        double x;
        double z0;
        double z1;
        unsigned passes = std::abs(cxxcam::units::length_mm(bbox.max.x - bbox.min.x).value()) / vm["stepdown"].as<double>();
        double step_x = cxxcam::units::length_mm(bbox.max.x - bbox.min.x).value() / static_cast<double>(passes);

        using namespace cxxcam::units;
        if (start_point.x == length_mm(bbox.min.x).value()) {
            x = cxxcam::units::length_mm(bbox.min.x).value();
        } else if (start_point.x == length_mm(bbox.max.x).value()) {
            x = cxxcam::units::length_mm(bbox.max.x).value();
            step_x = -step_x;
        } else {
            throw std::runtime_error("Initial X position must be located at min or max profile position");
        }

        if (start_point.z == length_mm(bbox.min.z).value()) {
            z0 = length_mm(bbox.min.z).value();
            z1 = length_mm(bbox.max.z).value();
        } else if (start_point.z == length_mm(bbox.max.z).value()) {
            z0 = length_mm(bbox.max.z).value();
            z1 = length_mm(bbox.min.z).value();
        } else {
            throw std::runtime_error("Initial Z position must be located at min or max profile position");
        }

        traverse_z(path, x, step_x, 0, passes+1, z0, z1);

/*        line_segment_2 sweep = {{x, length_mm(bbox.min.z).value()}, {x, length_mm(bbox.max.z).value()}};

        for (unsigned pass = 0; pass < passes; ++pass) {
        std::cerr << "x: " << x << "\n";
            sweep.a.x = sweep.b.x = x;
            auto intersections = intersects(path, sweep);
            for (auto idx : intersections) {
                auto line = path[idx];
                std::cerr << "x: " << x << " z: " << line.a.z << " " << line.b.z << "\n";
            }
            x += step_x;
        }*/

    } catch(const po::error& e) {
        print_exception(e);
        std::cout << options << "\n";
        return 1;
    } catch(const std::exception& e) {
        print_exception(e);
        return 1;
    }
}

