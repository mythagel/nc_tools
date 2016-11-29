#include "rs274_feedrate.h"
#include "rs274ngc_return.hh"
#include <boost/program_options.hpp>
#include "print_exception.h"
#include "base/machine_config.h"

#include <iostream>
#include <vector>
#include <string>
#include <lua.hpp>

/* TODO modify to optimise 2d feedrate only!
 * full 6axis (much!) later.
 *
 * stock as bounding box only (not model)
 * only 2d moves are analysed
 * reset when z level changes
 * for each z level, new slice of model (z delta stored from last for depth of cut calc)
 * 2d tool projection used for calculations, tool engagement determined from intersection
 * of tool line (open path) and model.
 * */

namespace po = boost::program_options;

int main(int argc, char* argv[]) {
    po::options_description options("nc_feedrate");
    std::vector<std::string> args(argv, argv + argc);
    args.erase(begin(args));

    options.add(machine_config::base_options());
    options.add_options()
        ("help,h", "display this help and exit")
        ("x0,X", po::value<double>()->default_value(0.0), "X0 dimension")
        ("y0,Y", po::value<double>()->default_value(0.0), "Y0 dimension")
        ("z0,Z", po::value<double>()->default_value(0.0), "Z0 dimension")
        ("x1,x", po::value<double>()->required(), "X1 dimension")
        ("y1,y", po::value<double>()->required(), "Y1 dimension")
        ("z1,z", po::value<double>()->required(), "Z1 dimension")
        ("tool", po::value<int>(), "Default tool")
        ("speed", po::value<double>(), "Spindle speed")
        ("slice_z", po::value<double>()->default_value(0.0003), "Z slice height")
    ;

    try {
        po::variables_map vm;
        store(po::command_line_parser(args).options(options).run(), vm);

        if(vm.count("help")) {
            std::cout << options << "\n";
            return 0;
        }
        notify(vm);

        double x0 = vm["x0"].as<double>();
        double y0 = vm["y0"].as<double>();
        double z0 = vm["z0"].as<double>();
        double x1 = vm["x1"].as<double>();
        double y1 = vm["y1"].as<double>();
        double z1 = vm["z1"].as<double>();

        cxxcam::Bbox stock;
        {
            using namespace cxxcam::units;
            stock.min.x = length{x0 * millimeters};
            stock.min.y = length{y0 * millimeters};
            stock.min.z = length{z0 * millimeters};

            stock.max.x = length{x1 * millimeters};
            stock.max.y = length{y1 * millimeters};
            stock.max.z = length{z1 * millimeters};
        }

        rs274_feedrate rate(vm, stock, vm["slice_z"].as<double>());

        if(vm.count("tool")) {
            std::stringstream s;
            s << "M06 T" << vm["tool"].as<int>();
            rate.read(s.str().c_str());
            rate.execute();
        }

        if(vm.count("speed")) {
            std::stringstream s;
            s << "M03 S" << vm["speed"].as<double>();
            rate.read(s.str().c_str());
            rate.execute();
        }

        std::string line;
        while(std::getline(std::cin, line)) {
            int status;

            status = rate.read(line.c_str());
            if(status != RS274NGC_OK) {
                if(status != RS274NGC_EXECUTE_FINISH) {
                    std::cerr << "Error reading line!: \n";
                    std::cerr << line <<"\n";
                    return status;
                }
            }
            
            status = rate.execute();
            if(status != RS274NGC_OK)
                return status;
            //std::cout << line << "\n";
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
