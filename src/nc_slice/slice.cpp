#include <boost/program_options.hpp>
#include "print_exception.h"
#include "geom/polyhedron.h"
#include "geom/query.h"
#include "geom/primitives.h"
#include "geom/ops.h"
#include "../throw_if.h"
#include "../r6.h"
#include "base/machine_config.h"

#include <iostream>
#include <vector>
#include <string>
#include <lua.hpp>

namespace po = boost::program_options;

int main(int argc, char* argv[]) {
    po::options_description options("nc_slice");
    std::vector<std::string> args(argv, argv + argc);
    args.erase(begin(args));

    options.add(machine_config::base_options());
    options.add_options()
        ("help,h", "display this help and exit")
        ("feedrate,f", po::value<double>()->required(), "Feed rate")
        ("stepdown,d", po::value<double>()->required(), "Z Stepdown")
    ;

    try {
        po::variables_map vm;
        store(po::command_line_parser(args).options(options).run(), vm);

        if(vm.count("help")) {
            std::cout << options << "\n";
            return 0;
        }
        notify(vm);

        double stepdown = vm["stepdown"].as<double>();
        double f = vm["feedrate"].as<double>();

        std::cerr << "reading model...";
        geom::polyhedron_t model;
        throw_if(!(std::cin >> geom::format::off >> model), "Unable to read model from file");
        std::cerr << "done.\n";

        std::cerr << "calculating bounds...";
        auto bbox = geom::bounding_box(model);
        std::cerr << "done.\n";
        double model_z = bbox.max.z - bbox.min.z;

        const unsigned n_steps = [&] {
            unsigned steps = std::abs(std::ceil(model_z / stepdown));
            return steps == 0 ? 1 : steps;
        }();
        const double step_z = model_z / n_steps;

        double z0 = bbox.max.z;
        for (unsigned step = 0; step < n_steps; ++step, z0 -= step_z) {
            double z1 = z0 - step_z;
            std::cerr << "Z: " << z1 << "\n";

            auto slice_bounds = geom::make_box({bbox.min.x, bbox.min.y, z1}, {bbox.max.x, bbox.max.y, z0});
            auto slice = geom::projection_xy(model * slice_bounds);

            auto output_slice = [&](const geom::polygon_t::polygon& path) {
                bool rapid_to_first = true;
                for (auto& p : path) {
                    if (rapid_to_first) {
                        std::cout << "G00 X" << r6(p.x) << " Y" << r6(p.y) << " Z" << r6(z1) << "\n";
                        rapid_to_first = false;
                    }
                    std::cout << "G01 X" << r6(p.x) << " Y" << r6(p.y) << " Z" << r6(z1) << " F" << r6(f) << "\n";
                }
                    std::cout << "G01 X" << r6(path.front().x) << " Y" << r6(path.front().y) << " Z" << r6(z1) << " F" << r6(f) << "\n";
            };

            for (auto& polygon : slice)
            {
                for (auto& p : polygon.polygons)
                    output_slice(p);
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

    return 0;
}
