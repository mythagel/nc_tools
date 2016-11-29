/* 
 * Copyright (C) 2016  Nicholas Gill
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * rs274_feedrate.cpp
 *
 *  Created on: 2016-02-22
 *      Author: nicholas
 */

#include "rs274_feedrate.h"
#include <cmath>
#include <cstring>
#include "cxxcam/Path.h"
#include <fstream>
#include "throw_if.h"
#include "fold_adjacent.h"
#include <iterator>
#include <algorithm>
#include <iostream>
#include "base/machine_config.h"
#include "formulas.h"

#include "clipper.hpp"
namespace cl = ClipperLib;
static const double CLIPPER_SCALE = 1e12;

namespace slicing {

namespace radius {
double endmill(double z, const machine_config::mill_tool& tool) {
    return (z > 0 && z <= tool.flute_length) ? tool.diameter / 2.0 : 0;
}
}

cl::Path endmill(double z, const machine_config::mill_tool& tool, unsigned segments = 64) {
    auto r = radius::endmill(z, tool);

    if (r == 0) return {};

    auto scale = [](double v) { return v * CLIPPER_SCALE; };

    cl::Path polygon;
    polygon.reserve(segments);

    double delta_theta = (2*PI) / segments;
    double theta = 0;
    for (unsigned i = 0; i < segments; ++i, theta += delta_theta)
        polygon.emplace_back(scale(std::cos(theta)*r), scale(std::sin(theta)*r));

    return polygon;
}

cl::Paths slice_bbox(double z, const cxxcam::Bbox& bbox) {
    using cxxcam::units::length_mm;

    cl::Paths paths;
    paths.emplace_back();
    auto& path = paths.back();

    auto scale_point = [](cxxcam::units::length x, cxxcam::units::length y) {
        return cl::IntPoint(length_mm(x).value() * CLIPPER_SCALE, length_mm(y).value() * CLIPPER_SCALE);
    };

    if (z < length_mm(bbox.min.z).value() || z > length_mm(bbox.max.z).value())
        return {};

    path.push_back(scale_point(bbox.min.x, bbox.min.y));
    path.push_back(scale_point(bbox.min.x, bbox.max.y));
    path.push_back(scale_point(bbox.max.x, bbox.max.y));
    path.push_back(scale_point(bbox.max.x, bbox.min.y));

    return paths;
}

}

void rs274_feedrate::_rapid(const Position& pos) {
    using namespace cxxcam;
	auto steps = path::expand_linear(convert(program_pos), convert(pos), {}, -1).path;

	auto length = path::length_linear(convert(program_pos), convert(pos));
    auto spindle_delta = spindle_delta_theta(length);
    apply_spindle_delta(spindle_delta);

    std::vector<bool> intersections;
    fold_adjacent(std::begin(steps), std::end(steps), std::back_inserter(intersections), 
		[this](const path::step& s0, const path::step& s1) -> bool
		{
            (void) s0;
            (void) s1;
            // TODO clipper minowski sum path + tool then check for intersections in this and all previous z levels
            return false;
		});

    if (std::find(begin(intersections), end(intersections), true) != end(intersections)) {
        // TODO better error reporting - annotate source command!
        std::cerr << "rapid intersection\n";
    }
}

/* TODO plan with z slices
 * store cl::Paths representing z slices of stock sorted by z height
 * need function to return cl::Paths representing tool model at particular z height - done
 * */
double rs274_feedrate::chip_load(const cxxcam::path::step& s0, const cxxcam::path::step& s1, double spindle_step) {
    using namespace cxxcam;
    using units::length_mm;

    auto cross = [](math::vector_3 v0, math::vector_3 v1) -> math::vector_3 {
        return {v0.y*v1.z - v0.z*v1.y, v0.z*v1.x - v0.x*v1.z, v0.x*v1.y - v0.y*v1.x, 0};
    };
    auto dot = [](math::vector_3 v0, math::vector_3 v1) -> double {
        return v0.x*v1.x + v0.y*v1.y + v0.z*v1.z;
    };

    auto to_quat = [&](math::vector_3 v0, math::vector_3 v1) -> math::quaternion_t {
        auto vec = normalise(cross(v0, v1));
        vec.a = acos(dot(v0, v1)) * 57.2958;
        return axis2quat(vec);
    };

    static const math::quaternion_t identity{1,0,0,0};

    const auto& o0 = s0.orientation;
    const auto& p0 = s0.position;
    const auto& p1 = s1.position;

    auto length = distance(p0, p1);

    /* for each piece of material removed
        * normalise orientation (reverse from tool orientation at point)
        * translate to origin at tool center
        * normalise orientation at z axis to normalise tool movement along x axis (arbitary)
        * calculate bounding box
        * height of material (bbox height) is depth of cut
        * calculate tool theta for step length at current rpm
        * bbox width on y axis is width of cut
        * */

/* TODO have different tools
 * one which calculates and annotates toolpath with comments
 * indicating the relevant measure parameters, chip load, cutter engagement
 * milling forces, etc,
 * and another which uses those to optimise feedrate
 * */

    auto scale_point = [](const cxxcam::math::point_3& p) {
        return cl::IntPoint(length_mm(p.x).value() * CLIPPER_SCALE, length_mm(p.y).value() * CLIPPER_SCALE);
    };

    auto debug_path = [](const cl::Paths& paths) {
        struct point { double x, y; };
        for(auto& path : paths) {
            auto unscale = [&](const cl::IntPoint& p) -> point {
                return {static_cast<double>(p.X) / CLIPPER_SCALE, static_cast<double>(p.Y) / CLIPPER_SCALE};
            };

            auto first = unscale(*path.begin());
            std::cout << "M " << first.x << " " << first.y << " ";
            for(auto& point : path) {
                auto p = unscale(point);
                std::cout << "L " << p.x << " " << p.y << " ";
            }
            std::cout << "L " << first.x << " " << first.y << " ";
        }
        std::cout << "\n";
    };

    double min_z = length_mm(p0.z).value();
    double max_z = length_mm(p1.z).value();
    for (auto& slice_desc : _slices) {
        auto slice_z0 = slice_desc.first;
        auto slice_z1 = slice_desc.first + _slice_z;
        auto& slice = slice_desc.second;
        if ((min_z >= slice_z0 && min_z <= slice_z1) ||
            (max_z >= slice_z0 && max_z <= slice_z1) ||
            (slice_z0 >= min_z && slice_z0 <= max_z) ||
            (slice_z1 >= min_z && slice_z1 <= max_z)) {

            auto tool_z = min_z - slice_z0;
            auto tool_slice = slicing::endmill(tool_z, _tool.mill);
            if (tool_slice.empty())
                continue;

            auto translate = [](const cl::Paths& paths, double x, double y) {
                cl::Paths translated;
                x *= CLIPPER_SCALE;
                y *= CLIPPER_SCALE;
                for (auto& path : paths) {
                    translated.emplace_back();
                    auto& t = translated.back();
                    for (auto& p : path)
                        t.emplace_back(p.X + x, p.Y + y);
                }
                return translated;
            };

            cl::Paths toolpath;
            //toolpath = translate({ tool_slice }, length_mm(p1.x).value(), length_mm(p1.y).value());
            MinkowskiSum(tool_slice, {scale_point(p0), scale_point(p1)}, toolpath, false);

            // calculate cutter engagement
            if (true)
            {
                cl::Clipper clipper;
                clipper.AddPaths(toolpath, cl::ptSubject, false);
                clipper.AddPaths(slice, cl::ptClip, true);

                cl::PolyTree pt;
                clipper.Execute(cl::ctIntersection, pt);

                cl::Paths open;
                OpenPathsFromPolyTree(pt, open);

                auto path_length = [](const cl::Paths& paths) {
                    double length = 0;
                    for (auto& path : paths) {
                        double x0 = static_cast<double>(path.front().X) / CLIPPER_SCALE;
                        double y0 = static_cast<double>(path.front().Y) / CLIPPER_SCALE;
                        for (auto& point : path) {
                            double x1 = static_cast<double>(point.X) / CLIPPER_SCALE;
                            double y1 = static_cast<double>(point.Y) / CLIPPER_SCALE;
                            length += std::sqrt(std::pow(x1 - x0, 2) + std::pow(y1 - y0, 2));
                            x0 = x1;
                            y0 = y1;
                        }
                    }
                    return length;
                };
                debug_path(open);
                {
                    auto l0 = path_length(toolpath);
                    auto l1 = path_length(open);
                    double engagement = 360.0 * (l1 / l0);

                    std::cerr << engagement << "\n";
                }
                // TODO
            }

            // remove toolpath from stock slice
            {
                cl::Clipper clipper;
                clipper.AddPaths(slice, cl::ptSubject, true);
                clipper.AddPaths(toolpath, cl::ptClip, true);
                slice.clear();
                clipper.Execute(cl::ctDifference, slice);

                //debug_path(slice);
            }


        }
    }

    // has to be done for EACH slice layer which potentially intersects tool
    // z == tool_z

    auto dir = math::vector_3{length_mm(p1.x - p0.x).value(), length_mm(p1.y - p0.y).value(), length_mm(p1.z - p0.z).value()};
    // TODO bail if z component of direction

    auto n_rotations = spindle_step / (2*PI);
    auto dist_per_rev = length_mm(length).value() / n_rotations;

    auto Vf = _feed_rate;
    auto n = _spindle_speed;
    auto Zc = _tool.mill.flutes;
    auto fz = formulas::fz(Vf, n, Zc);
    auto ae = 0;  // TODO cutter engagement
    auto ap = 0;  // TODO DOC
    auto Q = formulas::Q(ap, ae, Vf);
    //std::cerr << "Feed per tooth: " << fz << " mm\n";
    //std::cerr << "Material Removal Rate: " << Q << " cm3/min\n";

    return fz;
}

void rs274_feedrate::_arc(const Position& end, const Position& center, const cxxcam::math::vector_3& plane, int rotation) {
    using namespace cxxcam;
	auto steps = path::expand_arc(convert(program_pos), convert(end), convert(center), (rotation < 0 ? path::ArcDirection::Clockwise : path::ArcDirection::CounterClockwise), plane, std::abs(rotation), {}, 10).path;

	auto length = path::length_arc(convert(program_pos), convert(end), convert(center), (rotation < 0 ? path::ArcDirection::Clockwise : path::ArcDirection::CounterClockwise), plane, std::abs(rotation));
    auto spindle_delta = spindle_delta_theta(length);
    auto spindle_step = spindle_delta / (steps.size() - 1);

    std::vector<double> chip_load_per_tooth;
    fold_adjacent(std::begin(steps), std::end(steps), std::back_inserter(chip_load_per_tooth), 
		[&, this](const path::step& s0, const path::step& s1) -> double
		{
            return chip_load(s0, s1, spindle_step);
		});

    apply_spindle_delta(spindle_delta);
    //for(auto& clpt : chip_load_per_tooth) {
    //}
    // TODO analyse each toolpath step to determine appropriate feed rate
}


void rs274_feedrate::_linear(const Position& pos) {
    using namespace cxxcam;
	auto steps = path::expand_linear(convert(program_pos), convert(pos), {}, 10).path;

	auto length = path::length_linear(convert(program_pos), convert(pos));
    auto spindle_delta = spindle_delta_theta(length);
    auto spindle_step = spindle_delta / (steps.size() - 1);

    std::vector<double> chip_load_per_tooth;
    fold_adjacent(std::begin(steps), std::end(steps), std::back_inserter(chip_load_per_tooth), 
		[&, this](const path::step& s0, const path::step& s1) -> double
		{
            return chip_load(s0, s1, spindle_step);
		});

    apply_spindle_delta(spindle_delta);
}

void rs274_feedrate::tool_change(int slot) {
    using namespace machine_config;

    switch (get_machine_type(config, machine_id)) {
        case machine_type::mill: {
            mill_tool& t = _tool.mill;
            get_tool(config, slot, machine_id, t);
            // TODO ???
            break;
        }
        case machine_type::lathe: {
            lathe_tool &t = _tool.lathe;
            get_tool(config, slot, machine_id, t);
            // TODO
            break;
        }
    }
}

rs274_feedrate::rs274_feedrate(boost::program_options::variables_map& vm, const cxxcam::Bbox& stock, double slice_z)
 : rs274_base(vm), _stock(stock), _slice_z(slice_z) {
    using cxxcam::units::length_mm;
    double z_delta = length_mm(_stock.max.z - _stock.min.z).value();
    unsigned slices = std::abs(std::floor(z_delta / _slice_z));
    _slice_z = z_delta / slices;

    double z = length_mm(_stock.min.z).value();
    for (unsigned i = 0; i < slices; ++i) {
        _slices[z] = slicing::slice_bbox(z, _stock);
        z += _slice_z;
    }
}

