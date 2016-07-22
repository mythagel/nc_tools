#include "rs274ngc_return.hh"
#include <boost/program_options.hpp>
#include "print_exception.h"
#include "../throw_if.h"
#include "rs274_clipper_path.h"
#include <iostream>
#include "clipper.hpp"
#include "base/machine_config.h"
#include <algorithm>


namespace po = boost::program_options;
using namespace ClipperLib;

struct point_2
{
    double x;
    double y;
};
inline double distance(const point_2& a, const point_2& b) {
    return std::sqrt(std::pow(b.x - a.x, 2) + std::pow(b.y - a.y, 2));
}

int main(int argc, char* argv[]) {
    po::options_description options("nc_contour_pocket");
    std::vector<std::string> args(argv, argv + argc);
    args.erase(begin(args));

    options.add(machine_config::base_options());
    options.add_options()
        ("help,h", "display this help and exit")
        ("tool_r,r", po::value<double>()->required(), "Tool radius")
        ("stepover,s", po::value<double>()->default_value(0.9), "Tool stepover 0.0 - 1.0")
    ;

    try {
        po::variables_map vm;
        store(po::command_line_parser(args).options(options).run(), vm);

        if(vm.count("help")) {
            std::cout << options << "\n";
            return 0;
        }
        notify(vm);

        rs274_clipper_path nc_path(vm);
        double tool_offset = vm["tool_r"].as<double>() * 2 * vm["stepover"].as<double>();

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
        point_2 current_point = {0, 0};
        while (true) {
            Paths solution;
            co.Execute(solution, offset * nc_path.scale());

            auto to_point = [&](const IntPoint& p) -> point_2 {
                return {static_cast<double>(p.X)/nc_path.scale(), static_cast<double>(p.Y)/nc_path.scale()};
            };

            for(auto& path : solution) {

                // TODO sort subpaths

                auto it_near = std::min_element(begin(path), end(path), 
                    [&](const IntPoint& l, const IntPoint& r) -> bool {
                        return std::abs(distance(to_point(l), current_point)) < std::abs(distance(to_point(r), current_point));
                    });
                std::rotate(begin(path), it_near, end(path));

                std::cout << std::fixed << "G00 X" << static_cast<double>(path.begin()->X)/nc_path.scale() << " Y" << static_cast<double>(path.begin()->Y)/nc_path.scale() << "\n";
                for(auto& p : path) {
                    std::cout << std::fixed << "G01 X" << static_cast<double>(p.X)/nc_path.scale() << " Y" << static_cast<double>(p.Y)/nc_path.scale() << " F50\n";
                }
                std::cout << std::fixed << "G01 X" << static_cast<double>(path.begin()->X)/nc_path.scale() << " Y" << static_cast<double>(path.begin()->Y)/nc_path.scale() << "\n";
                std::cout << "\n";
                current_point = to_point(*path.begin());
            }

            if(solution.empty())
                break;

            offset -= tool_offset;
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

