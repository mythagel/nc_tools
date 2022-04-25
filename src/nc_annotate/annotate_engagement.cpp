#include "rs274_annotate.h"
#include "rs274ngc_return.hh"
#include <boost/program_options.hpp>
#include "print_exception.h"
#include "base/machine_config.h"
#include "geom/polyhedron.h"
#include "geom/ops.h"
#include "geom/query.h"
#include "geom/primitives.h"
#include "clipper.hpp"
#include "../throw_if.h"
#include "../r6.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <utility>
#include <map>

namespace po = boost::program_options;
namespace cl = ClipperLib;

cl::Paths generate_tool(double r, double x, double y, double scale, unsigned segments = 64) {
    cl::Path polygon;
    polygon.reserve(segments);

    double delta_theta = (2*PI) / segments;
    double theta = 0;
    for (unsigned i = 0; i < segments; ++i, theta += delta_theta)
        polygon.emplace_back(((std::cos(theta)*r)+x) * scale, ((std::sin(theta)*r)+y) * scale);

    return { polygon };
}


class rs274_annotate_engagement : public rs274_annotate
{
private:
	void process_point(const cxxcam::math::point_3& p, bool rapid)
    {
        using cxxcam::units::length_mm;

        if (rapid) {
            last_point_valid = false;
            return;
        }

        double z0 = last_z;
        double z1 = length_mm(p.z).value();
        if (z1 > z0)
            std::swap(z0, z1);

        // Update last z height
 //       last_z = length_mm(p.z).value();

        double scale = 10e12;
        auto scale_point = [scale] (const geom::polygon_t::point& point){
            return cl::IntPoint(point.x * scale, point.y * scale);
        };
        auto unscale_point = [&](const cl::IntPoint& p) -> geom::polygon_t::point {
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

        if (last_point_valid == false) {
            last_point = scale_point({length_mm(p.x).value(), length_mm(p.y).value()});
            last_point_valid = true;
            return;
        }

        // TODO  2.5d only? how to handle general points
        cl::Paths* slice = nullptr;
        auto key = std::make_pair(static_cast<long>(z0 * scale), static_cast<long>(z1 * scale));

        auto it = slices.find(key);
        if (it != slices.end()) {
            slice = &it->second;
        } else {
            auto slice_bounds = geom::make_box({bbox.min.x, bbox.min.y, z1}, {bbox.max.x, bbox.max.y, z0});
            auto stock_slice = geom::projection_xy(stock * slice_bounds);
            auto res = slices.emplace(key, scale_paths(stock_slice));
            slice = &res.first->second;
        }

        unsigned tool_points = 64;
        double tool_r = 2;
        auto tool_paths = generate_tool(tool_r, length_mm(p.x).value(), length_mm(p.y).value(), scale, tool_points);

        auto tool_point = [=](const cl::IntPoint& p) -> bool {
            for (auto& path : tool_paths)
                if (std::find(begin(path), end(path), p) != end(path))
                    return true;
            return false;
        };

        // minkowski_sum to last point
        cl::MinkowskiSum(tool_paths[0], {last_point, scale_point({length_mm(p.x).value(), length_mm(p.y).value()})}, tool_paths, false);
        last_point = scale_point({length_mm(p.x).value(), length_mm(p.y).value()});

        cl::Clipper clpr;
        clpr.AddPaths(*slice, cl::ptSubject, true);
        clpr.AddPaths(tool_paths, cl::ptClip, true);

        cl::Paths engagement_path;
        clpr.Execute(cl::ctIntersection, engagement_path);

        clpr.Execute(cl::ctDifference, *slice);

        unsigned engagement_points = 0;
        for (auto& path : engagement_path) {
            for(auto& point : path) {
                if (!tool_point(point)) continue;
                ++engagement_points;
            }
        }

        fprintf(stderr, "blah: %d\n", engagement_points);

        double engagement = static_cast<double>(engagement_points) / tool_points;
        if (engagement_points == 1)
            engagement = -tool_r * 2;    // TODO technically plunge?!
        else
            engagement = (tool_r * 4) * (engagement > 0.5 ? 0.5 : engagement); // 0.5 == full diameter
        
        if (engagement > 0) {
            std::ostringstream ss;
            ss << "ae" << r6(engagement);     // Working engagement
            push_annotation(ss.str());
        }
    }

    geom::polyhedron_t stock;
    geom::query::bbox_3 bbox;
    double last_z;

    cl::IntPoint last_point;
    bool last_point_valid = false;

    std::map<std::pair<long, long>, cl::Paths> slices;
public:
	rs274_annotate_engagement(boost::program_options::variables_map& vm, const geom::polyhedron_t& stock)
        : rs274_annotate(vm), stock(stock)
    {
        bbox = geom::bounding_box(stock);
        last_z = bbox.max.z;
    }
	virtual ~rs274_annotate_engagement() = default;
};

int main(int argc, char* argv[]) {
    po::options_description options("nc_annotate_engagement");
    std::vector<std::string> args(argv, argv + argc);
    args.erase(begin(args));

    options.add(machine_config::base_options());
    options.add_options()
        ("help,h", "display this help and exit")
        ("stock", po::value<std::string>()->required(), "Stock model file")
    ;

    try {
        po::variables_map vm;
        store(po::command_line_parser(args).options(options).run(), vm);

        if(vm.count("help")) {
            std::cout << options << "\n";
            return 0;
        }
        notify(vm);

        geom::polyhedron_t stock;
        std::ifstream is(vm["stock"].as<std::string>());
        throw_if(!(is >> geom::format::off >> stock), "Unable to read stock from file");

        rs274_annotate_engagement annotate(vm, stock);

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
