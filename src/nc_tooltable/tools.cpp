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

        std::cout << ";\n";
        for(unsigned slot = 0; slot < 56; ++slot) {
            using namespace cxxcam::gcode;
            auto& L = config.state();

            lua_getglobal(L, "tool_table");
            if (!lua_istable(L, -1)) {
                lua_pop(L, 1);
                continue;
            }

            lua_pushinteger(L, slot);
            lua_gettable(L, -2);
            if (!lua_istable(L, -1)) {
                lua_pop(L, 2);
                continue;
            }
            
            Line line;
            line += {Word::T, static_cast<double>(slot)};
            line += {Word::P, static_cast<double>(slot)};

            lua_getfield(L, -1, "diameter");
            if(lua_isnumber(L, -1))
                line += {Word::D, lua_tonumber(L, -1)};
            lua_pop(L, 1);

            lua_getfield(L, -1, "name");
            if(lua_isstring(L, -1))
                line.Comment(lua_tostring(L, -1));
            lua_pop(L, 1);


            std::copy(line.begin(), line.end(), std::ostream_iterator<Word>(std::cout, " "));
            if(!line.Comment().empty())
                std::cout << "; " << line.Comment();
            std::cout << "\n";

            lua_pop(L, 1);
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
