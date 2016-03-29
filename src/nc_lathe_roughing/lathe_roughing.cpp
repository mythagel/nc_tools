#include "rs274ngc_return.hh"
#include <boost/program_options.hpp>
#include "print_exception.h"
#include "../throw_if.h"
#include "rs274_path.h"

#include <iostream>
#include <vector>
#include <string>
#include "geometry.h"

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

int main(int argc, char* argv[]) {
    po::options_description options("nc_lathe_roughing");
    std::vector<std::string> args(argv, argv + argc);
    args.erase(begin(args));

    options.add_options()
        ("help,h", "display this help and exit")
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
        {
            line_segment_2 line;
            line.a = {0,0};
            line.b = {0,1};

            ray_2 ray;
            ray.p = {0.5, 0.5};
            ray.q = {0, 0.5};

            if(auto p = intersects(line, ray)) {
                std::cout << "INTERSECTS " << p->x << ", " << p->y << "\n";
            } else {
                std::cout << "no intersection\n";
            }

            arc_2 arc;

            /*
            arc.dir = arc_2::cw;
            arc.a = {-5,0};
            arc.b = {5,0};
            arc.c = {0,0};
            */
            arc.dir = arc_2::ccw;
            arc.a = {0,5};
            arc.b = {5,0};
            arc.c = {0,0};

            for(int y1 = 110; y1 >= -10; --y1) {
                double y = (y1 / 10.0) -5;

                ray_2 ray2;
                ray2.p = {0, y};
                ray2.q = {1, y};

                std::cout << y <<" ";
                auto p = intersects(arc, ray2);
                if(p.first) {
                    std::cout << "INTERSECTION1 " << p.first->x << ", " << p.first->y << "\n";
                }
                if(p.second) {
                    std::cout << "INTERSECTION2 " << p.second->x << ", " << p.second->y << "\n";
                }
                if(!p.first && !p.second) {
                    std::cout << "no intersection\n";
                }
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

