#include "rs274_feedrate.h"
#include "rs274ngc_return.hh"
#include <boost/program_options.hpp>
#include "print_exception.h"
#include "base/machine_config.h"

#include <iostream>
#include <vector>
#include <string>
#include <lua.hpp>

namespace po = boost::program_options;

int main(int argc, char* argv[]) {
    po::options_description options("nc_feedrate");
    std::vector<std::string> args(argv, argv + argc);
    args.erase(begin(args));

    options.add(machine_config::base_options());
    options.add_options()
        ("help,h", "display this help and exit")
        ("stock", po::value<std::string>()->required(), "Stock model file")
        ("tool", po::value<int>(), "Default tool")
    ;

    try {
        po::variables_map vm;
        store(po::command_line_parser(args).options(options).run(), vm);

        if(vm.count("help")) {
            std::cout << options << "\n";
            return 0;
        }
        notify(vm);

        rs274_feedrate rate(vm, vm["stock"].as<std::string>());

        if(vm.count("tool")) {
            std::stringstream s;
            s << "M06 T" << vm["tool"].as<int>();
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
            std::cout << line << "\n";
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
