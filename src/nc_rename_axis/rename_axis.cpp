#include "rs274_rename.h"
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
    po::options_description options("nc_rename_axis");
    std::vector<std::string> args(argv, argv + argc);
    args.erase(begin(args));

    options.add(machine_config::base_options());
    options.add_options()
        ("help,h", "display this help and exit")
        ("delete,d", po::value<AxisModification>(), "delete axis [XYZABC]")
        ("swap,s", po::value<AxisModification>(), "swap axes [XYZABC][XYZABC]")
    ;

    try {
        auto parsed = po::command_line_parser(args).options(options).run();

        po::variables_map vm;
        store(parsed, vm);

        for (auto& option : parsed.options) {
            if(option.string_key == "help") {
                std::cout << options << "\n";
                return 0;
            }
        }

        std::vector<AxisModification> mods;
        auto is_none = [](AxisModification::Axis a) { return a == AxisModification::axis_None; };
        for (auto& option : parsed.options) {
            if (option.string_key == "delete") {
                auto value = boost::lexical_cast<AxisModification>(option.value[0]);
                if(is_none(value.from) || !is_none(value.to))
                    throw std::runtime_error("Invalid axis specification for delete option");
                mods.push_back(value);
            } else if (option.string_key == "swap") {
                auto value = boost::lexical_cast<AxisModification>(option.value[0]);
                if(is_none(value.from) || is_none(value.to))
                    throw std::runtime_error("Invalid axis specification for swap option");
                mods.push_back(value);
            }
        }

        rs274_rename rename(vm, mods);

        std::string line;
        while(std::getline(std::cin, line)) {
            int status;

            status = rename.read(line.c_str());
            if(status != RS274NGC_OK) {
                if(status != RS274NGC_EXECUTE_FINISH) {
                    std::cerr << "Error reading line!: \n";
                    std::cout << line <<"\n";
                    return status;
                }
            }
            
            status = rename.execute();
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
