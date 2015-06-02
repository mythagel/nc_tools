#include "rs274_model.h"
#include "rs274ngc_return.hh"
#include <boost/program_options.hpp>

#include <iostream>
#include <lua.hpp>

namespace po = boost::program_options;

int main(int argc, char* argv[]) {
    po::options_description options("nc_model");
    std::vector<std::string> args(argv, argv + argc);
    args.erase(begin(args));

    options.add_options()
        ("help,h", "display this help and exit")
        ("stock", "Stock model file")
    ;

    rs274_model modeler;

    std::string line;
    while(std::getline(std::cin, line)) {
        int status;

        status = modeler.read(line.c_str());
        if(status != RS274NGC_OK) {
            if(status != RS274NGC_EXECUTE_FINISH) {
                std::cerr << "Error reading line!: \n";
                std::cerr << line <<"\n";
                return status;
            }
        }
        
        status = modeler.execute();
        if(status != RS274NGC_OK)
            return status;
    }

    return 0;
}
