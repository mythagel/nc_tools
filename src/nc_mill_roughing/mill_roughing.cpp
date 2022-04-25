#include <boost/program_options.hpp>
#include "print_exception.h"
#include "geom/polyhedron.h"
#include "geom/query.h"
#include "geom/primitives.h"
#include "geom/ops.h"
#include "geom/translate.h"
#include "../throw_if.h"
#include "../r6.h"
#include "base/machine_config.h"
#include "clipper.hpp"
#include "cxxcam/Math.h"

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <lua.hpp>

namespace po = boost::program_options;
namespace cl = ClipperLib;
using namespace cxxcam;

void fold_path(const cl::PolyNode* node, cl::Paths& paths) {
    paths.push_back(node->Contour);
    for (auto& child : node->Childs)
        fold_path(child, paths);
}

int main(int argc, char* argv[]) {
    po::options_description options("nc_mill_roughing");
    std::vector<std::string> args(argv, argv + argc);
    args.erase(begin(args));

    options.add(machine_config::base_options());
    options.add_options()
        ("help,h", "display this help and exit")
        ("feedrate,f", po::value<double>()->required(), "Feed rate")    // TODO calculate
        ("stepdown,d", po::value<double>()->required(), "Z Stepdown")
        ("rotate_x,a", po::value<unsigned>(), "Flip N time around X axis")
        ("rotate_y,b", po::value<unsigned>(), "Flip N time around Y axis")
 //       ("rotary", "Use rotary axis for repositioning")
    ;

    try {
        po::variables_map vm;
        store(po::command_line_parser(args).options(options).run(), vm);

        throw_if (vm.count("rotate_x") && vm.count("rotate_y"), "Conflicting options. Specify rotate_x or rotate_y");

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

        // Align Z0 to top of model
        // NOPE - align midpoint of model to center
//        model = geom::translate(model, 0, 0, -bbox.max.z);
//        bbox = geom::bounding_box(model);



        unsigned N = 1;
        if (vm.count("rotate_x"))
            N = vm["rotate_x"].as<unsigned>();
        if (vm.count("rotate_y"))
            N = vm["rotate_y"].as<unsigned>();
        for (unsigned i = 0; i < N; ++i) {
            double angle = (360.0/N) * i;

            geom::polyhedron_t oriented_model = model;
            if(vm.count("rotate_x")) {
                auto theta = units::plane_angle{angle * units::degrees};
                auto q = math::normalise(math::axis2quat(1, 0, 0, theta));
                oriented_model = geom::rotate(oriented_model, q.R_component_1(), q.R_component_2(), q.R_component_3(), q.R_component_4());
            } else if(vm.count("rotate_y")) {
                auto theta = units::plane_angle{angle * units::degrees};
                auto q = math::normalise(math::axis2quat(0, 1, 0, theta));
                oriented_model = geom::rotate(oriented_model, q.R_component_1(), q.R_component_2(), q.R_component_3(), q.R_component_4());
            }

           // TODO flip, align, slice
            auto bbox = geom::bounding_box(oriented_model);
            double model_z = bbox.max.z - bbox.min.z;

            auto stock_box = geom::make_box({bbox.min.x, bbox.min.y, 0}, {bbox.max.x, bbox.max.y, 1});
            cl::Paths stock = scale_paths(geom::projection_xy(stock_box));
            //cl::Paths stock = scale_paths(geom::projection_xy(oriented_model));

            const unsigned n_steps = [&] {
                unsigned steps = std::abs(std::ceil(model_z / stepdown));
                return steps == 0 ? 1 : steps;
            }();
            const double step_z = model_z / n_steps;

            double z0 = bbox.max.z;
            for (unsigned step = 0; step < n_steps; ++step, z0 -= step_z) {
                double slice_z0 = bbox.max.z;
                double z1 = z0 - step_z;

                auto slice_bounds = geom::make_box({bbox.min.x, bbox.min.y, z1}, {bbox.max.x, bbox.max.y, slice_z0});
                std::vector<geom::polygon_tree_t> slice;

                {
                    auto model_slice = scale_paths(geom::projection_xy(oriented_model * slice_bounds));
                    cl::Paths model_stock;

                    cl::Clipper clipper;
                    clipper.AddPaths(stock, cl::ptSubject, true);
                    cl::PolyTree pt;
                    clipper.Execute(cl::ctUnion, pt);
                    for (auto& child : pt.Childs) {
                        cl::Paths paths;
                        fold_path(child, paths);

                        // TODO eliminating the outer path leaves the inner paths without a border...
                        //if (! paths.empty())
                        //    model_stock.insert(end(model_stock), begin(paths)+1, end(paths));
                        model_stock.insert(end(model_stock), begin(paths), end(paths));
                    }


                    // TODO
                    // generate internal stock by interesecting with external path only
                    // subtract internal paths ONLY from intersected stock
                    // do external paths LAST
                    cl::Clipper clpr;
                    clpr.AddPaths(model_stock, cl::ptSubject, true);
                    clpr.AddPaths(model_slice, cl::ptClip, true);

                    cl::Paths solution;
                    clpr.Execute(cl::ctDifference, solution);

                    for (auto path : solution) {
                        geom::polygon_tree_t poly;
                        poly.polygons.emplace_back();
                        for (auto point : path) {
                            poly.polygons.back().points.push_back(unscale_point(point));
                        }
                        slice.push_back(poly);
                    }
                }

                auto output_path = [&](const geom::polygon_t& path) {
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
                    for (auto& path : polygon.polygons)
                        output_path(path);
                }
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
