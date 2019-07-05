#include "rs274_annotate.h"
#include "rs274ngc_return.hh"
#include <boost/program_options.hpp>
#include "print_exception.h"
#include "base/machine_config.h"
#include "circle3d.hpp"
#include "../r6.h"

#include <iostream>
#include <sstream>
#include <vector>
#include <string>

namespace po = boost::program_options;

class rs274_annotate_curvature : public rs274_annotate
{
private:
	void process_point(const cxxcam::math::point_3& p, bool rapid)
    {
        using cxxcam::units::length_mm;

        if (rapid) {
            // TODO flush annotations?
            points.clear();
            return;
        }

        points.push_back(p);

        if (points.size() >= 3) {
            
            if (points.size() > 3)
                points.erase(begin(points));

            circle3d::Point3D p1 = {length_mm(points[0].x).value(), length_mm(points[0].y).value(), length_mm(points[0].z).value()};
            circle3d::Point3D p2 = {length_mm(points[1].x).value(), length_mm(points[1].y).value(), length_mm(points[1].z).value()};
            circle3d::Point3D p3 = {length_mm(points[2].x).value(), length_mm(points[2].y).value(), length_mm(points[2].z).value()};
            circle3d::Point3D c;
            double radius;
            circle3d::estimate3DCircle(p1, p2, p3, c, radius);

            double curve = 1/radius;

            if (isnan(curve) == false) {
                std::ostringstream ss;
                ss << "R" << r6(curve);     // Radius of curvature
                push_annotation(ss.str());
            }
        }
    }

    std::vector<cxxcam::math::point_3> points;

public:
	rs274_annotate_curvature(boost::program_options::variables_map& vm)
        : rs274_annotate(vm) {}
	virtual ~rs274_annotate_curvature() = default;
};

int main(int argc, char* argv[]) {
    po::options_description options("nc_annotate");
    std::vector<std::string> args(argv, argv + argc);
    args.erase(begin(args));

    options.add(machine_config::base_options());
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

        rs274_annotate_curvature annotate(vm);

        std::string line;
        while(std::getline(std::cin, line)) {
            int status;

            status = annotate.read(line.c_str());
            if(status != RS274NGC_OK) {
                if(status != RS274NGC_EXECUTE_FINISH) {
                    std::cerr << "Error reading line!: \n";
                    return status;
                }
            }
            
            status = annotate.execute();
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
