#include "rs274_arcfit.h"
#include "rs274ngc_return.hh"
#include <boost/program_options.hpp>
#include "print_exception.h"
#include "../throw_if.h"

#include <iostream>
#include <vector>
#include <string>

namespace po = boost::program_options;

int main(int argc, char* argv[]) {
    po::options_description options("nc_arcfit");
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

        rs274_arcfit arcfit;

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
