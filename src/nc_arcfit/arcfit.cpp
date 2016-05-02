#include "rs274_arcfit.h"
#include "rs274ngc_return.hh"
#include <boost/program_options.hpp>
#include "print_exception.h"
#include "../throw_if.h"

#include <iostream>
#include <vector>
#include <string>
#include "base/machine_config.h"

namespace po = boost::program_options;

int main(int argc, char* argv[]) {
    po::options_description options("nc_arcfit");
    std::vector<std::string> args(argv, argv + argc);
    args.erase(begin(args));

    options.add(machine_config::base_options());
    options.add_options()
        ("help,h", "display this help and exit")
        ("chord_height,c", po::value<double>()->default_value(0.1), "Chord height tolerance")
        ("radius_dev,r", po::value<double>()->default_value(0.1), "Radius deviation tolerance")
        ("planar_dev,p", po::value<double>()->default_value(1e-6), "Planar deviation tolerance")
        ("theta_min,t", po::value<double>()->default_value(3.14/16.0), "Minimum arc theta")
    ;

    try {
        po::variables_map vm;
        store(po::command_line_parser(args).options(options).run(), vm);

        if(vm.count("help")) {
            std::cout << options << "\n";
            return 0;
        }
        notify(vm);

        double chord_height_tolerance = vm["chord_height"].as<double>();
        double point_deviation = vm["radius_dev"].as<double>();
        double planar_tolerance = vm["planar_dev"].as<double>();
        double theta_minimum = vm["theta_min"].as<double>();

        rs274_arcfit arcfit(vm, chord_height_tolerance, point_deviation, planar_tolerance, theta_minimum);

        std::string line;
        while(std::getline(std::cin, line)) {
            int status;

            status = arcfit.read(line.c_str());
            if(status != RS274NGC_OK) {
                if(status != RS274NGC_EXECUTE_FINISH) {
                    std::cerr << "Error reading line!: \n";
                    std::cout << line <<"\n";
                    return status;
                }
            }
            
            status = arcfit.execute();
            if(status != RS274NGC_OK)
                return status;
        }

        if (arcfit.read("M2") == RS274NGC_OK)
            arcfit.execute();

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
