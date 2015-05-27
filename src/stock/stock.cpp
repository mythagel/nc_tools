#include "geom/polyhedron.h"
#include "geom/primitives.h"
#include <iostream>
#include <boost/program_options.hpp>
#include <vector>
#include <string>
#include "print_exception.h"

namespace po = boost::program_options;

int main(int argc, char* argv[]) {
    po::options_description options("nc_stock");
    std::vector<std::string> args(argv, argv + argc);
    args.erase(begin(args));

    options.add_options()
        ("help,h", "display this help and exit")
        ("box", "Box stock shape")
        (",x", po::value<double>()->required(), "X dimension")
        (",y", po::value<double>()->required(), "Y dimension")
        (",z", po::value<double>()->required(), "Z dimension")
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
            auto stock = geom::make_box({x:0, y:0, z:0}, {x:vm[",x"].as<double>(), y:vm[",y"].as<double>(), z:vm[",z"].as<double>()});
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
