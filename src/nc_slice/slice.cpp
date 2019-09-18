#include <boost/program_options.hpp>
#include "print_exception.h"
#include "geom/polyhedron.h"
#include "geom/query.h"
#include "geom/primitives.h"
#include "geom/ops.h"
#include "../throw_if.h"
#include "../r6.h"
#include "base/machine_config.h"
#include "clipper.hpp"

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <lua.hpp>

namespace po = boost::program_options;
namespace cl = ClipperLib;

int main(int argc, char* argv[]) {
    po::options_description options("nc_slice");
    std::vector<std::string> args(argv, argv + argc);
    args.erase(begin(args));

    options.add(machine_config::base_options());
    options.add_options()
        ("help,h", "display this help and exit")
        ("feedrate,f", po::value<double>()->required(), "Feed rate")
        ("stepdown,d", po::value<double>()->required(), "Z Stepdown")
        ("stock", po::value<std::string>(), "Stock model file")
        ("offset,o", po::value<double>(), "Generate stock via offset")
    ;

    try {
        po::variables_map vm;
        store(po::command_line_parser(args).options(options).run(), vm);

        if(vm.count("help")) {
            std::cout << options << "\n";
            return 0;
        }
        notify(vm);

        double stepdown = vm["stepdown"].as<double>();
        double f = vm["feedrate"].as<double>();

        double scale = 10e12;
        auto scale_point = [scale] (const geom::polygon_t::point& point){
            return cl::IntPoint(point.x * scale, point.y * scale);
        };
        auto unscale_point = [scale](const cl::IntPoint& p) -> geom::polygon_t::point {
            return {static_cast<double>(p.X)/scale, static_cast<double>(p.Y)/scale};
        };
        auto scale_paths = [scale_point] (const std::vector<geom::polygon_tree_t>& polygons) {
            cl::Paths paths;
            for (auto& polygon : polygons) {
                for (auto& poly : polygon.polygons) {
                    paths.emplace_back();
                    for (auto& p : poly.points)
                        paths.back().push_back(scale_point(p));
                }
            }
            return paths;
        };

        geom::polyhedron_t model;
        throw_if(!(std::cin >> geom::format::off >> model), "Unable to read model from file");

        geom::polyhedron_t stock;
        bool has_stock_file = vm.count("stock");
        if (has_stock_file) {
            std::ifstream is(vm["stock"].as<std::string>());
            throw_if(!(is >> geom::format::off >> stock), "Unable to read stock from file");
        }

        cl::Paths stock_offset;
        bool has_stock_offset = vm.count("offset");
        if (has_stock_offset)
        {
            stock_offset = scale_paths(geom::projection_xy(model));

            cl::ClipperOffset co;
            co.AddPaths(stock_offset, cl::jtRound, cl::etClosedPolygon);
            co.ArcTolerance = 0.1 * scale;
            co.Execute(stock_offset, vm["offset"].as<double>() * scale);
        }

        auto bbox = geom::bounding_box(model);
        double model_z = bbox.max.z - bbox.min.z;

        const unsigned n_steps = [&] {
            unsigned steps = std::abs(std::ceil(model_z / stepdown));
            return steps == 0 ? 1 : steps;
        }();
        const double step_z = model_z / n_steps;

        double z0 = bbox.max.z;
        for (unsigned step = 0; step < n_steps; ++step, z0 -= step_z) {
            double z1 = z0 - step_z;

            auto slice_bounds = geom::make_box({bbox.min.x, bbox.min.y, z1}, {bbox.max.x, bbox.max.y, z0});
            auto slice = geom::projection_xy(model * slice_bounds);

            // If stock (model or offset) has been provided, calculate the slice of material to be removed
            if (has_stock_file || has_stock_offset) {

                cl::Clipper clpr;
                clpr.AddPaths(scale_paths(slice), cl::ptClip, true);

                if (has_stock_file) {
                    auto stock_slice = geom::projection_xy(stock * slice_bounds);
                    clpr.AddPaths(scale_paths(stock_slice), cl::ptSubject, true);
                }
                else if (has_stock_offset) {
                    clpr.AddPaths(stock_offset, cl::ptSubject, true);
                }

                cl::Paths solution;
                clpr.Execute(cl::ctDifference, solution);

                slice.clear();
                for (auto path : solution) {
                    geom::polygon_tree_t poly;
                    poly.polygons.emplace_back();
                    for (auto point : path) {
                        poly.polygons.back().points.push_back(unscale_point(point));
                    }
                    slice.push_back(poly);
                }
            }

            auto output_slice = [&](const geom::polygon_t& path) {
                bool rapid_to_first = true;
                for (auto& p : path.points) {
                    if (rapid_to_first) {
                        std::cout << "G00 X" << r6(p.x) << " Y" << r6(p.y) << " Z" << r6(z1) << "\n";
                        rapid_to_first = false;
                    }
                    std::cout << "G01 X" << r6(p.x) << " Y" << r6(p.y) << " Z" << r6(z1) << " F" << r6(f) << "\n";
                }
                    std::cout << "G01 X" << r6(path.points.front().x) << " Y" << r6(path.points.front().y) << " Z" << r6(z1) << " F" << r6(f) << "\n";
            };

            for (auto& polygon : slice)
            {
                for (auto& p : polygon.polygons)
                    output_slice(p);
            }
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
