#include "rs274_bounds.h"
#include "rs274ngc_return.hh"
#include <boost/program_options.hpp>
#include "print_exception.h"
#include "geom/polyhedron.h"
#include "geom/query.h"
#include "../throw_if.h"
#include "../r6.h"

#include <iostream>
#include <vector>
#include <string>
#include <lua.hpp>

namespace po = boost::program_options;

std::ostream& operator<<(std::ostream& os, const geom::query::bbox_3& b) {
    os << "min: {" << r6(b.min.x) << ", " << r6(b.min.y) << ", " << r6(b.min.z) <<"} max: {" << r6(b.max.x) << ", " << r6(b.max.y) << ", " << r6(b.max.z) <<"}";
    return os;
}

int main(int argc, char* argv[]) {
    po::options_description options("nc_bounds");
    std::vector<std::string> args(argv, argv + argc);
    args.erase(begin(args));

    options.add_options()
        ("help,h", "display this help and exit")
        ("cut,c", "track cuts in gcode")
        ("rapid,r", "track rapids in gcode")
        ("model,m", "calculate bounding box of model")
    ;

    try {
        po::variables_map vm;
        store(po::command_line_parser(args).options(options).run(), vm);

        if(vm.count("help")) {
            std::cout << options << "\n";
            return 0;
        }
        notify(vm);

        if (vm.count("model")) {
            geom::polyhedron_t model;
            throw_if(!(std::cin >> geom::format::off >> model), "Unable to read model from file");

            auto bounding_box = geom::bounding_box(model);
            std::cout << bounding_box << "\n";

        } else {
            bool cut = vm.count("cut");
            bool rapid = vm.count("rapid");
            if(! (cut || rapid))
                cut = true;
            rs274_bounds bounding_box(cut, rapid);

            std::string line;
            while(std::getline(std::cin, line)) {
                int status;

                status = bounding_box.read(line.c_str());
                if(status != RS274NGC_OK) {
                    if(status != RS274NGC_EXECUTE_FINISH) {
                        std::cerr << "Error reading line!: \n";
                        std::cout << line <<"\n";
                        return status;
                    }
                }
                
                status = bounding_box.execute();
                if(status != RS274NGC_OK)
                    return status;
            }

            std::cout << bounding_box.bounding_box() << "\n";
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
