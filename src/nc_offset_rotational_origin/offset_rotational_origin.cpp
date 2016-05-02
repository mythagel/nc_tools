#include "rs274_offset.h"
#include "rs274ngc_return.hh"
#include <boost/program_options.hpp>
#include "print_exception.h"
#include "../throw_if.h"
#include "base/machine_config.h"

#include <iostream>
#include <vector>
#include <string>

namespace po = boost::program_options;

int main(int argc, char* argv[]) {
    po::options_description options("nc_offset_rotational_origin");
    std::vector<std::string> args(argv, argv + argc);
    args.erase(begin(args));

    options.add(machine_config::base_options());
    options.add_options()
        ("help,h", "display this help and exit")
        ("from-tool,t", "From - Tool zero as rotational origin")
        ("from-x,x", po::value<double>(), "From - X axis rotational origin")
        ("from-y,y", po::value<double>(), "From - Y axis rotational origin")
        ("from-z,z", po::value<double>(), "From - Z axis rotational origin")
        ("to-tool,T", "To - Tool zero as rotational origin")
        ("to-x,X", po::value<double>(), "To - X axis rotational origin")
        ("to-y,Y", po::value<double>(), "To - Y axis rotational origin")
        ("to-z,Z", po::value<double>(), "To - Z axis rotational origin")
    ;

    try {
        po::variables_map vm;
        store(po::command_line_parser(args).options(options).run(), vm);

        if(vm.count("help")) {
            std::cout << options << "\n";
            return 0;
        }
        notify(vm);

        rotational_origin from;
        rotational_origin to;

        if(vm.count("from-tool"))
            from = {};
        else
            from = { vm["from-x"].as<double>(), vm["from-y"].as<double>(), vm["from-z"].as<double>() };

        if(vm.count("to-tool"))
            from = rotational_origin();
        else
            from = { vm["to-x"].as<double>(), vm["to-y"].as<double>(), vm["to-z"].as<double>() };


        rs274_offset offset(vm, from, to);

        std::string line;
        while(std::getline(std::cin, line)) {
            int status;

            status = offset.read(line.c_str());
            if(status != RS274NGC_OK) {
                if(status != RS274NGC_EXECUTE_FINISH) {
                    std::cerr << "Error reading line!: \n";
                    return status;
                }
            }
            
            status = offset.execute();
            if(status != RS274NGC_OK)
                return status;
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
