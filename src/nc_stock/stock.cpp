#include "geom/polyhedron.h"
#include "geom/primitives.h"
#include <iostream>
#include <boost/program_options.hpp>
#include "print_exception.h"
#include <vector>
#include <string>
#include "../throw_if.h"

namespace po = boost::program_options;

int main(int argc, char* argv[]) {
    po::options_description options("nc_stock");
    std::vector<std::string> args(argv, argv + argc);
    args.erase(begin(args));

    options.add_options()
        ("help,h", "display this help and exit")
        ("box", "Box stock shape")
        ("cylinder", "Cylinder stock shape")
        ("x0,X", po::value<double>()->default_value(0.0), "X0 dimension")
        ("y0,Y", po::value<double>()->default_value(0.0), "Y0 dimension")
        ("z0,Z", po::value<double>()->default_value(0.0), "Z0 dimension")
        ("x1,x", po::value<double>()->required(), "X1 dimension")
        ("y1,y", po::value<double>()->required(), "Y1 dimension")
        ("z1,z", po::value<double>()->required(), "Z1 dimension")
        ("radius,r", po::value<double>(), "Cylinder radius")
        ("segments,S", po::value<unsigned>()->default_value(64), "Cylinder segments")
    ;

    try {
        po::variables_map vm;
        store(po::command_line_parser(args).options(options).run(), vm);

        if(vm.count("help")) {
            std::cout << options << "\n";
            return 0;
        }
        notify(vm);

        if(vm.count("box")) {
            double x0 = vm["x0"].as<double>();
            double y0 = vm["y0"].as<double>();
            double z0 = vm["z0"].as<double>();
            double x1 = vm["x1"].as<double>();
            double y1 = vm["y1"].as<double>();
            double z1 = vm["z1"].as<double>();
            auto stock = geom::make_box({x0, y0, z0}, {x1, y1, z1});
            std::cout << geom::format::off << stock;
        } else if(vm.count("cylinder")) {
            throw_if(!vm.count("radius"), "radius required for cylinder stock");

            double x0 = vm["x0"].as<double>();
            double y0 = vm["y0"].as<double>();
            double z0 = vm["z0"].as<double>();
            double x1 = vm["x1"].as<double>();
            double y1 = vm["y1"].as<double>();
            double z1 = vm["z1"].as<double>();
            double r = vm["radius"].as<double>();
            double s = vm["segments"].as<unsigned>();
            auto stock = geom::make_cone({x0, y0, z0}, {x1, y1, z1}, r, r, s);            
            std::cout << geom::format::off << stock;
        } else {
            std::cerr << "must specify shape\n";
            return 1;
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
