#include "rs274ngc_return.hh"
#include <boost/program_options.hpp>
#include "print_exception.h"
#include "../throw_if.h"
#include "rs274_clipper_path.h"
#include <iostream>
#include "clipper.hpp"


namespace po = boost::program_options;
using namespace ClipperLib;

int main(int argc, char* argv[]) {
    po::options_description options("nc_contour_pocket");
    std::vector<std::string> args(argv, argv + argc);
    args.erase(begin(args));

    options.add_options()
        ("help,h", "display this help and exit")
        ("tool_r,r", po::value<unsigned>()->required(), "Tool radius")
    ;

    try {
        po::variables_map vm;
        store(po::command_line_parser(args).options(options).run(), vm);

        if(vm.count("help")) {
            std::cout << options << "\n";
            return 0;
        }
        notify(vm);

        rs274_clipper_path nc_path;
        unsigned tool_r = vm["tool_r"].as<unsigned>();

        // TODO read default init line from nc_tools.conf
//        nc_path.read("G18");
//        nc_path.execute();

        std::string line;
        while(std::getline(std::cin, line)) {
            int status;

            status = nc_path.read(line.c_str());
            if(status != RS274NGC_OK) {
                if(status != RS274NGC_EXECUTE_FINISH) {
                    std::cerr << "Error reading line!: \n";
                    std::cout << line <<"\n";
                    return status;
                }
            }
            
            status = nc_path.execute();
            if(status != RS274NGC_OK)
                return status;
        }

        auto paths = nc_path.path();

        ClipperOffset co;
        for (auto& path : paths)
            co.AddPath(path, jtRound, etClosedPolygon);
        co.ArcTolerance = 0.1 * nc_path.scale();

        double offset = 0.0;
        while (true) {
            Paths solution;
            co.Execute(solution, offset * nc_path.scale());

            // TODO find point on path closest to current point and rotate
            // points so that one is first
            for(auto& path : solution) {
                std::cout << std::fixed << "G00 X" << static_cast<double>(path.begin()->X)/nc_path.scale() << " Y" << static_cast<double>(path.begin()->Y)/nc_path.scale() << "\n";
                for(auto& p : path) {
                    std::cout << std::fixed << "G01 X" << static_cast<double>(p.X)/nc_path.scale() << " Y" << static_cast<double>(p.Y)/nc_path.scale() << " F50\n";
                }
                std::cout << std::fixed << "G01 X" << static_cast<double>(path.begin()->X)/nc_path.scale() << " Y" << static_cast<double>(path.begin()->Y)/nc_path.scale() << "\n";
                std::cout << "\n";
            }

            if(solution.empty())
                break;

            offset -= tool_r;
        }

    } catch(const po::error& e) {
        print_exception(e);
        std::cout << options << "\n";
        return 1;
    } catch(const std::exception& e) {
        print_exception(e);
        return 1;
    }
}

