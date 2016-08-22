#include "rs274_delay.h"
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
    po::options_description options("nc_delay");
    std::vector<std::string> args(argv, argv + argc);
    args.erase(begin(args));

    options.add(machine_config::base_options());
    options.add_options()
        ("help,h", "display this help and exit")
        ("scale,s", po::value<double>()->default_value(1.0), "feed rate scale factor")
        ("measure,m", "measure only")
    ;

    try {
        po::variables_map vm;
        store(po::command_line_parser(args).options(options).run(), vm);

        if(vm.count("help")) {
            std::cout << options << "\n";
            return 0;
        }
        notify(vm);

        rs274_delay delayer(vm, vm["scale"].as<double>(), vm.count("measure"));

        std::string line;
        while(std::getline(std::cin, line)) {
            int status;

            status = delayer.read(line.c_str());
            if(status != RS274NGC_OK) {
                if(status != RS274NGC_EXECUTE_FINISH) {
                    std::cerr << "Error reading line!: \n";
                    std::cout << line <<"\n";
                    return status;
                }
            }
            
            status = delayer.execute();
            if(status != RS274NGC_OK)
                return status;
            std::cout << line << "\n";
        }

        auto duration = delayer.cut_duration();
        auto hours = std::chrono::duration_cast<std::chrono::hours>(duration);
        auto minutes = std::chrono::duration_cast<std::chrono::minutes>(duration - hours);
        auto seconds = std::chrono::duration_cast<std::chrono::seconds>(duration - (hours + minutes));
        auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(duration - (hours + minutes + seconds));
        if (hours.count() > 0) std::cerr << hours.count() << " h ";
        if (minutes.count() > 0) std::cerr << minutes.count() << " m ";
        if (seconds.count() > 0) std::cerr << seconds.count() << " s ";
        if (milliseconds.count() > 0) std::cerr << milliseconds.count() << " ms ";
        std::cerr << "\n";

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
