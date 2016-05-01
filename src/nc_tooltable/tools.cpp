#include <boost/program_options.hpp>
#include "print_exception.h"

#include <iostream>
#include <vector>
#include <string>
#include <lua.hpp>

#include "GCodeLine.h"
#include <algorithm>
#include <iterator>
#include "base/nc_config.h"
#include "base/machine_config.h"

namespace po = boost::program_options;

/* http://wiki.linuxcnc.org/cgi-bin/wiki.pl?ToolTable
 * ; - opening semicolon, no data following 
 * T - tool number, 0-99999 (you can have a large number of tools in inventory) 
 * P - pocket number, 1-99999 (tool table has a lower number of entries, currently 56.) 
 * X..W - tool offset on specified axis - floating-point 
 * D - tool diameter - floating-point 
 * I - front angle (lathe only) - floating-point 
 * J - back angle (lathe only) - floating-point 
 * Q - tool orientation (lathe only) - integer, 0-9 
 * ; - beginning of comment or remark - text */
int main(int argc, char* argv[]) {
    po::options_description options("nc_tooltable");
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

        nc_config config;
        auto machine_id = machine_config::default_machine(config);
        auto type = machine_config::get_machine_type(config, machine_id);

        std::cout << ";\n";
        for(unsigned slot = 0; slot < 56; ++slot) {
            using namespace cxxcam::gcode;
            using namespace machine_config;

            Line line;
            line += {Word::T, static_cast<double>(slot)};
            line += {Word::P, static_cast<double>(slot)};

            switch (type) {
                case machine_type::mill: {
                    mill_tool tool;
                    if (!get_tool(config, slot, machine_id, tool))
                        continue;

                    line += {Word::D, tool.diameter};
                    line.Comment(tool.name);
                    break;
                }
                case machine_type::lathe: {
                    // TODO
                    break;
                }
            }

            std::copy(line.begin(), line.end(), std::ostream_iterator<Word>(std::cout, " "));
            if(!line.Comment().empty())
                std::cout << "; " << line.Comment();
            std::cout << "\n";
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
