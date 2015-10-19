#include "geom/polyhedron.h"
#include "geom/translate.h"
#include <iostream>
#include <boost/program_options.hpp>
#include "print_exception.h"
#include <vector>
#include <string>
#include "../throw_if.h"
#include "Math.h"

namespace po = boost::program_options;
using namespace cxxcam;

int main(int argc, char* argv[]) {
    po::options_description options("nc_transform");
    std::vector<std::string> args(argv, argv + argc);
    args.erase(begin(args));

    options.add_options()
        ("help,h", "display this help and exit")
        ("translate_x,x", po::value<double>(), "Translate along x axis")
        ("translate_y,y", po::value<double>(), "Translate along y axis")
        ("translate_z,z", po::value<double>(), "Translate along z axis")
        ("rotate_x,a", po::value<double>(), "Rotate around x axis")
        ("rotate_y,b", po::value<double>(), "Rotate around y axis")
        ("rotate_z,c", po::value<double>(), "Rotate around z axis")
    ;

    try {
        auto parsed = po::command_line_parser(args).options(options).run();

        for (auto& option : parsed.options) {
            if(option.string_key == "help") {
                std::cout << options << "\n";
                return 0;
            }
        }

        geom::polyhedron_t model;
        throw_if(!(std::cin >> geom::format::off >> model), "Unable to read model from file");

        // apply transformations in order
        for (auto& option : parsed.options) {
            auto value = boost::lexical_cast<double>(option.value[0]);
            if(option.string_key == "translate_x") {
                model = geom::translate(model, value, 0, 0);
            } else if(option.string_key == "translate_y") {
                model = geom::translate(model, 0, value, 0);
            } else if(option.string_key == "translate_z") {
                model = geom::translate(model, 0, 0, value);
            } else if(option.string_key == "rotate_x") {
                auto theta = units::plane_angle{value * units::degrees};
                auto q = math::normalise(math::axis2quat(1, 0, 0, theta));
                model = geom::rotate(model, q.R_component_1(), q.R_component_2(), q.R_component_3(), q.R_component_4());
            } else if(option.string_key == "rotate_y") {
                auto theta = units::plane_angle{value * units::degrees};
                auto q = math::normalise(math::axis2quat(0, 1, 0, theta));
                model = geom::rotate(model, q.R_component_1(), q.R_component_2(), q.R_component_3(), q.R_component_4());
            } else if(option.string_key == "rotate_z") {
                auto theta = units::plane_angle{value * units::degrees};
                auto q = math::normalise(math::axis2quat(0, 0, 1, theta));
                model = geom::rotate(model, q.R_component_1(), q.R_component_2(), q.R_component_3(), q.R_component_4());
            }
        }

        std::cout << model;

    } catch(const po::error& e) {
        print_exception(e);
        std::cout << options << "\n";
        return 1;
    } catch(const std::exception& e) {
        print_exception(e);
        return 1;
    }
    return 0;
}
