#include <iostream>
#include <boost/program_options.hpp>
#include "print_exception.h"
#include <vector>
#include <string>
#include "svg_path/path.h"
#include "../r6.h"
#include <cmath>

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/io.hpp>

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

        /* 
         * generate sequence of biarcs based on acceptable max error
         * ALWAYS generate arcs, use nc_shortlines to generate lines if necessary
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

#define DUMP(x) std::cerr << std::fixed << #x << ": " << x << "\n"
	virtual void elliptical_arc_to(bool abs, float rx, float ry, float x_rotation, bool large_arc, bool sweep, float x, float y) {
        std::cerr << "args------------------------\n";
        x = abs ? x : pos.x + x;
        y = abs ? y : pos.y + y;
        using namespace boost::numeric::ublas;
        double pi = 3.14159265359;

        double phi = std::fmod(x_rotation, 360.0) * (pi / 180.0);
        double x1 = pos.x;
        double y1 = pos.y;

        DUMP(rx);
        DUMP(ry);
        DUMP(x_rotation);
        DUMP(large_arc);
        DUMP(sweep);
        DUMP(x);
        DUMP(y);
        DUMP(phi);
        DUMP(x1);
        DUMP(y1);
        std::cerr << "x1y1_prime------------------------\n";
        matrix<double> x1y1_prime(2,1);
        {
            matrix<double> m0(2, 2);
            m0(0,0) = std::cos(phi);
            m0(0,1) = std::sin(phi);
            m0(1,0) = -std::sin(phi);
            m0(1,1) = std::cos(phi);
            DUMP(m0);

            matrix<double> m1(2, 1);
            m1(0,0) = (x1-x)/2.0;
            m1(1,0) = (y1-y)/2.0;
            DUMP(m1);

            x1y1_prime = prod(m0, m1);
            DUMP(x1y1_prime);
        }

        std::cerr << "c_prime------------------------\n";
        matrix<double> c_prime(2,1);
        {
            double x1_prime = x1y1_prime(0,0);
            double y1_prime = x1y1_prime(1,0);

            matrix<double> m0(2, 1);
            m0(0,0) = (rx * y1_prime) / ry;
            m0(1,0) = -(ry * x1_prime) / rx;
            DUMP(m0);

            auto rx2 = rx*rx;
            auto ry2 = ry*ry;
            auto x1p2 = x1_prime*x1_prime;
            auto y1p2 = y1_prime*y1_prime;
            DUMP(rx2);
            DUMP(ry2);
            DUMP(x1p2);
            DUMP(y1p2);
            double s0_num = (rx2 * ry2) - (rx2 * y1p2) - (ry2 * x1p2);
            double s0_den = (rx2 * y1p2) + (ry2 * x1p2);
            DUMP(s0_num);
            DUMP(s0_den);
            DUMP(s0_num/s0_den);

            auto sign = (large_arc != sweep ? 1.0 : -1.0);
            DUMP(sign);

            c_prime = (sign * std::sqrt(std::abs(s0_num / s0_den))) * m0;
            DUMP(c_prime);
        }
        
        std::cerr << "c------------------------\n";
        matrix<double> c(2,1);
        {
            matrix<double> m0(2, 2);
            m0(0,0) = std::cos(phi);
            m0(0,1) = -std::sin(phi);
            m0(1,0) = std::sin(phi);
            m0(1,1) = std::cos(phi);
            DUMP(m0);

            matrix<double> m1(2, 1);
            m1(0,0) = (x1+x)/2.0;
            m1(1,0) = (y1+y)/2.0;
            DUMP(m1);

            c = prod(m0, c_prime) + m1;
        }
        DUMP(c);

        auto delta_angle = [](vector<double> u, vector<double> v) -> double {
            DUMP(u);
            DUMP(v);
            auto sign = u(0)*v(1) - u(1)*v(0);
            DUMP(sign);

            auto num = inner_prod(u, v);
            auto den = norm_2(u) * norm_2(v);
            DUMP(num);
            DUMP(den);
            DUMP(num/den);
            return std::abs(std::acos(num/den)) * (sign >= 0 ? 1 : -1);
        };

        std::cerr << "theta1------------------------\n";
        double theta1 = 0;
        {
            double x1_prime = x1y1_prime(0,0);
            double y1_prime = x1y1_prime(1,0);
            double cx_prime = c_prime(0,0);
            double cy_prime = c_prime(1,0);

            vector<double> u(2);
            u(0) = 1;
            u(1) = 0;

            vector<double> v(2);
            v(0) = (x1_prime - cx_prime) / rx;
            v(1) = (y1_prime - cy_prime) / ry;
            
            theta1 = delta_angle(u, v);
            DUMP(theta1);
        }

        std::cerr << "delta_theta------------------------\n";
        double delta_theta = 0;
        {
            double x1_prime = x1y1_prime(0,0);
            double y1_prime = x1y1_prime(1,0);
            double cx_prime = c_prime(0,0);
            double cy_prime = c_prime(1,0);

            vector<double> u(2);
            u(0) = (x1_prime - cx_prime) / rx;
            u(1) = (y1_prime - cy_prime) / ry;

            vector<double> v(2);
            v(0) = (-x1_prime - cx_prime) / rx;
            v(1) = (-y1_prime - cy_prime) / ry;

            delta_theta = delta_angle(u, v);

            if (!sweep && delta_theta > 0)
                delta_theta -= 2 * pi;
            else if(sweep && delta_theta < 0)
                delta_theta += 2 * pi;
            DUMP(delta_theta);
        }

        matrix<double> m0(2, 2);
        m0(0,0) = std::cos(phi);
        m0(0,1) = -std::sin(phi);
        m0(1,0) = std::sin(phi);
        m0(1,1) = std::cos(phi);

        auto step = delta_theta / 100.0;
        double theta = theta1;
        for(unsigned _ = 0; _ < 100; ++_, theta += step) {

            matrix<double> m1(2, 1);
            m1(0,0) = rx * std::cos(theta);
            m1(1,0) = ry * std::sin(theta);

            matrix<double> p(2, 1);
            p = prod(m0, m1) + c;
            std::cout << "G01 X" << r6(p(0,0)) << " Y" << r6(p(1,0)) << " F" << r6(f) << "\n";
        }


        pos.x = x;
        pos.y = y;
        // x_rotation is in degrees
        /*std::cerr << "elliptical arc\n";
        for(float t = 0.0; t < 2*pi; t += 0.01) {
            x = rx * std::cos(t);
            y = ry * std::sin(t);
            std::cout << "G01 X" << r6(x) << " Y" << r6(y) << " F" << r6(f) << "\n";
        }*/
        std::cerr << "\n";
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
