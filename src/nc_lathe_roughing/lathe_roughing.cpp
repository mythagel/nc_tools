#include "rs274ngc_return.hh"
#include <boost/program_options.hpp>
#include "print_exception.h"
#include "../throw_if.h"
#include "rs274_path.h"

#include <iostream>
#include <vector>
#include <string>
#include "geometry.h"

#include "Bbox.h"
#include "Units.h"

namespace po = boost::program_options;

class intersect_visitor : public boost::static_visitor<boost::optional<rs274_path::geometry>> {
private:
    ray_2 ray;
public:
    explicit intersect_visitor(const ray_2& ray) : ray(ray) {}

    boost::optional<rs274_path::geometry> operator()(const line_segment_2& line) const {
        if (intersects(line, ray))
            return { {line} };
        return {};
    }
    boost::optional<rs274_path::geometry> operator()(const arc_2& arc) const {
        auto i = intersects(arc, ray);
        if (i.first || i.second)
            return { {arc} };
        return {};
    }
};
std::vector<rs274_path::geometry> intersects(const std::vector<rs274_path::geometry>& path, const ray_2& ray) {
    std::vector<rs274_path::geometry> intersections;
    intersect_visitor visitor(ray);
    for (auto& geom : path) {
        if (auto result = boost::apply_visitor( visitor, geom )) {
            intersections.push_back(*result);
        }
    }
    return intersections;
}

template <typename Fn>
void arc_points(const arc_2& arc, Fn&& fn) {
    auto r = std::abs(distance(arc.c, arc.a));
    auto pi2 = 2.0 * 3.14159265359;
    auto start_theta = theta(arc, arc.a);
    auto end_theta = theta(arc, arc.b);
    auto delta_theta = end_theta - start_theta;
    switch(arc.dir) {
        case arc_2::cw:
            if (delta_theta > 0)
                delta_theta -= pi2;
            else if (delta_theta == 0)
                delta_theta = -pi2;
            break;
        case arc_2::ccw:
            if (delta_theta < 0)
                delta_theta += pi2;
            else if (delta_theta == 0)
                delta_theta = pi2;
            break;
    }
    unsigned total_steps = static_cast<unsigned>((128.0 / pi2) * std::abs(delta_theta));
    auto t = start_theta;
    auto dt = delta_theta / total_steps;
    if (delta_theta < 0) dt = -dt;
    for(size_t s = 0; s < total_steps; ++s, t += dt) {
        point_2 p;
        p.x = (std::cos(t) * r) + arc.c.x;
        p.z = (std::sin(t) * r) + arc.c.z;

        fn(p);
    }
}

class bounding_box_visitor : public boost::static_visitor<void> {
public:
    cxxcam::Bbox bbox;
    cxxcam::math::point_3 pos2point(const point_2& p) {
        using namespace cxxcam::units;

        cxxcam::math::point_3 cp;
        cp.x = length{p.x * millimeters};
        cp.z = length{p.z * millimeters};
        return cp;
    }

    void operator()(const line_segment_2& line) {
        bbox += pos2point(line.a);
        bbox += pos2point(line.b);
    }
    void operator()(const arc_2& arc) {
        arc_points(arc, [&](const point_2& p) {
            bbox += pos2point(p);
        });
    }
};

class monotonic_visitor : public boost::static_visitor<void> {
public:
    bool monotonic_x = true;
    bool monotonic_y = true;
    point_2 p = {0, 0};

    void operator()(const line_segment_2& line) {
        if(line.a.x < p.x) monotonic_x = false;
        if(line.a.z < p.z) monotonic_y = false;
        p = line.a;

        if(line.b.x < p.x) monotonic_x = false;
        if(line.b.z < p.z) monotonic_y = false;
        p = line.b;
    }
    void operator()(const arc_2& arc) {
        arc_points(arc, [&](const point_2& ap) {
            if(ap.x < p.x) monotonic_x = false;
            if(ap.z < p.z) monotonic_y = false;
            p = ap;
        });
    }
};

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

        auto path = nc_path.path();

        monotonic_visitor monotonic;
        bounding_box_visitor bbox;
        for (auto& geom : path) {
            boost::apply_visitor(monotonic, geom);
            boost::apply_visitor(bbox, geom);
        }
        // std::cout << "x: " << monotonic.monotonic_x << " y: " << monotonic.monotonic_y << "\n";
        std::cout << bbox.bbox << "\n";

    } catch(const po::error& e) {
        print_exception(e);
        std::cout << options << "\n";
        return 1;
    } catch(const std::exception& e) {
        print_exception(e);
        return 1;
    }

}

