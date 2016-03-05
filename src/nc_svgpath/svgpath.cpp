#include <iostream>
#include <boost/program_options.hpp>
#include "print_exception.h"
#include <vector>
#include <string>
#include "svg_path/path.h"
#include "../r6.h"

namespace po = boost::program_options;
namespace path = svg::types::parsers::path;

struct point_t {
    double x;
    double y;
    point_t(double x=0, double y=0)
        : x(x), y(y)
    {}
};
point_t lerp(const point_t& p0, const point_t& p1, float t) {
    return { (1-t)*p0.x + t*p1.x, (1-t)*p0.y + t*p1.y };
}

class nc_svg : public path::parser
{
private:
    double f;

    point_t pos;
    point_t subpath_start;

    point_t curve_cp;
    point_t bezier_cp;
protected:
    virtual void move_to(bool abs, float x, float y) {
        pos.x = abs ? x : pos.x + x;
        pos.y = abs ? y : pos.y + y;

        subpath_start = pos;
        std::cout << "G00 X" << r6(pos.x) << " Y" << r6(pos.y) << "\n";
    }

    virtual void line_to(bool abs, float x, float y) {
        pos.x = abs ? x : pos.x + x;
        pos.y = abs ? y : pos.y + y;
        curve_cp = bezier_cp = pos;
        std::cout << "G01 X" << r6(pos.x) << " Y" << r6(pos.y) << " F" << r6(f) << "\n";
    }
    virtual void horizontal_line_to(bool abs, float x) {
        pos.x = abs ? x : pos.x + x;
        curve_cp = bezier_cp = pos;
        std::cout << "G01 X" << r6(pos.x) << " F" << r6(f) << "\n";
    }
    virtual void vertical_line_to(bool abs, float y) {
        pos.y = abs ? y : pos.y + y;
        curve_cp = bezier_cp = pos;
        std::cout << "G01 Y" << r6(pos.y) << " F" << r6(f) << "\n";
    }

    virtual void curve_to(bool abs, float x1, float y1, float x2, float y2, float x, float y) {
        point_t p[4];
        p[0] = pos;
        p[1] = {abs ? x1 : pos.x + x1, abs ? y1 : pos.y + y1};
        p[2] = {abs ? x2 : pos.x + x2, abs ? y2 : pos.y + y2};
        p[3] = {abs ? x : pos.x + x, abs ? y : pos.y + y};
        pos = bezier_cp = p[3];
        curve_cp = p[2];

        /* TODO approximate arc length to calculate number of points to generate
         * */
        for(float t = 0.0; t < 1.0; t += 0.03) {
            auto ab = lerp(p[0], p[1], t);
            auto bc = lerp(p[1], p[2], t);
            auto cd = lerp(p[2], p[3], t);
            auto point = lerp(lerp(ab, bc, t), lerp(bc, cd, t), t);

            std::cout << "G01 X" << r6(point.x) << " Y" << r6(point.y) << " F" << r6(f) << "\n";
        }
        std::cout << "G01 X" << r6(pos.x) << " Y" << r6(pos.y) << " F" << r6(f) << "\n";
    }
    virtual void smooth_curve_to(bool abs, float x2, float y2, float x, float y) {
        float x1 = pos.x + (pos.x - curve_cp.x);
        float y1 = pos.y + (pos.y - curve_cp.y);
        if (abs)
            curve_to(true, x1, y1, x2, y2, x, y);
        else
            curve_to(true, x1, y1, pos.x+x2, pos.y+y2, pos.x+x, pos.y+y);
    }

    virtual void bezier_curve_to(bool abs, float x1, float y1, float x, float y) {
        point_t p[3];
        p[0] = pos;
        p[1] = {abs ? x1 : pos.x + x1, abs ? y1 : pos.y + y1};
        p[2] = {abs ? x : pos.x + x, abs ? y : pos.y + y};
        pos = curve_cp = p[2];
        bezier_cp = p[1];

        /* TODO approximate arc length to calculate number of points to generate
         * */
        for(float t = 0.0; t < 1.0; t += 0.03) {
            auto ab = lerp(p[0], p[1], t);
            auto bc = lerp(p[1], p[2], t);
            auto point = lerp(ab, bc, t);

            std::cout << "G01 X" << r6(point.x) << " Y" << r6(point.y) << " F" << r6(f) << "\n";
        }
        std::cout << "G01 X" << r6(pos.x) << " Y" << r6(pos.y) << " F" << r6(f) << "\n";
    }
    virtual void smooth_bezier_curve_to(bool abs, float x, float y) {
        float x1 = pos.x + (pos.x - bezier_cp.x);
        float y1 = pos.y + (pos.y - bezier_cp.y);
        if (abs)
            bezier_curve_to(true, x1, y1, x, y);
        else
            bezier_curve_to(true, x1, y1, pos.x+x, pos.y+y);
    }

    virtual void close_path() {
        pos = subpath_start;
        std::cout << "G01 X" << r6(pos.x) << " Y" << r6(pos.y) << " F" << r6(f) << "\n";
    }
    virtual void eof() {
        pos = {};
        subpath_start = {};
        curve_cp = bezier_cp = {};
    }

public:

    nc_svg(double f) : f(f) {
    }
    virtual ~nc_svg() =default;
};

int main(int argc, char* argv[]) {
    po::options_description options("nc_stock");
    std::vector<std::string> args(argv, argv + argc);
    args.erase(begin(args));

    options.add_options()
        ("help,h", "display this help and exit")
        ("feedrate,f", po::value<double>()->required(), "cutting feed rate")
    ;

    try {
        po::variables_map vm;
        store(po::command_line_parser(args).options(options).run(), vm);

        if(vm.count("help")) {
            std::cout << options << "\n";
            return 0;
        }
        notify(vm);

        nc_svg svg(vm["feedrate"].as<double>());

        std::string line;
        while(std::getline(std::cin, line)) {
            svg.parse(line.c_str(), line.c_str()+line.size());
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
