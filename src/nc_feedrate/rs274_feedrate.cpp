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
#include "Path.h"
#include "Simulation.h"
#include <fstream>
#include "throw_if.h"
#include "geom/primitives.h"
#include "fold_adjacent.h"
#include "geom/ops.h"
#include <iterator>
#include <algorithm>
#include "geom/query.h"
#include "geom/translate.h"
#include <iostream>
#include "base/machine_config.h"

#include "../r6.h"
std::ostream& operator<<(std::ostream& os, const geom::query::bbox_3& b) {
    os << "min: {" << r6(b.min.x) << ", " << r6(b.min.y) << ", " << r6(b.min.z) <<"} max: {" << r6(b.max.x) << ", " << r6(b.max.y) << ", " << r6(b.max.z) <<"}";
    return os;
}

namespace formulas {
// http://www.sandvik.coromant.com/en-us/knowledge/milling/formulas_and_definitions/formulas

/* Dcap/mm      - Cutter diameter at actual depth of cut
 * fz/mm        - feed per tooth
 * Zn           - total cutter teeth
 * Zc           - effective cutter teeth
 * Vf/mm/min    - table feed
 * fn/mm        - feed per revolution
 * ap/mm        - depth of cut
 * Vc/m/min     - Cutting speed
 * Y0           - chip rake angle
 * ae/mm        - working engagement
 * n/rpm        - spindle speed
 * Pc/kW        - net power
 * Mc/Nm        - Torque
 * Q/cm3/min    - Material removal rate
 * hm/mm        - Average chip thickness
 * hex/mm       - Max chip thickness
 * Kr/deg       - Entering angle
 * Dm/mm        - Machined diameter (component diameter)
 * Dw/mm        - Unmachined diameter
 * Vfm/mm/min   - Table feed of tool at Dm (machined diameter)
 */

double Vc(double Dcap, double n) {
    return (Dcap * PI * n) / 1000.0;
}

double n(double Vc, double Dcap) {
    return (Vc * 1000.0) / (PI * Dcap);
}

double fz(double Vf, double n, unsigned Zc) {
    return Vf / (n * Zc);
}

double Q(double ap, double ae, double Vf) {
    return (ap * ae * Vf) / 1000.0;
}

double Vf(double fz, double n, double Zc) {
    return fz * n * Zc;
}

double Mc(double Pc, double n) {
    return (Pc * 30.0 * 1000.0) / (PI * n);
}

double Pc(double ap, double ae, double Vf, double kc) {
    return (ap * ae * Vf * kc) / (60 * 1000000.0);
}

double hm_side(double Kr, double ae, double fz, double Dcap) {
    auto deg2rad = [](double d) { return (d / 180.0) * PI; };
    return (360 * std::sin(deg2rad(Kr)) * ae * fz) / (PI * Dcap * std::acos(deg2rad(1- ((2 * ae) / Dcap) )));
}
double hm_face(double Kr, double ae, double fz, double Dcap) {
    auto deg2rad = [](double d) { return (d / 180.0) * PI; };
    return (180 * std::sin(deg2rad(Kr)) * ae * fz) / (PI * Dcap * std::asin(deg2rad(ae/Dcap)));
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
			auto toolpath = simulation::sweep_tool(_toolmodel + _tool_shank, s0, s1);
            return intersects(toolpath, _model);
		});

    if (std::find(begin(intersections), end(intersections), true) != end(intersections)) {
        // TODO better error reporting - annotate source command!
        std::cerr << "rapid intersection\n";
    }
}

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
    auto tool_path = simulation::sweep_tool(_toolmodel, s0, s1);
    if(!intersects(tool_path, _model))
        return 0.0;

    auto mat = _model * tool_path;
    _model -= tool_path;
    auto deorient = identity;
    deorient /= o0;

    /* TODO perhaps instead of reorienting, fire ray in direction of travel and measure distance between entry and exit point.
     * will work for plunge (direction of travel == z), normal xy travel (widest part of chip), & 3d moves */

    auto dir = math::vector_3{length_mm(p1.x - p0.x).value(), length_mm(p1.y - p0.y).value(), length_mm(p1.z - p0.z).value()};
    // ignore z component of direction
    dir.z = 0;
    auto reorient = identity;
    reorient /= to_quat(normalise(dir), {1, 0, 0});

    mat = translate(mat, length_mm(-p0.x).value(), length_mm(-p0.y).value(), length_mm(-p0.z).value());
    mat = rotate(mat, deorient.R_component_1(), deorient.R_component_2(), deorient.R_component_3(), deorient.R_component_4());
    mat = rotate(mat, reorient.R_component_1(), reorient.R_component_2(), reorient.R_component_3(), reorient.R_component_4());

    auto n_rotations = spindle_step / (2*PI);
    auto dist_per_rev = length_mm(length).value() / n_rotations;

    auto bbox = bounding_box(mat);
    auto x_width = bbox.max.x - bbox.min.x;
    auto y_width = bbox.max.y - bbox.min.y;
    auto z_width = bbox.max.z - bbox.min.z;

    auto Vf = _feed_rate;
    auto n = _spindle_speed;
    auto Zc = _tool.mill.flutes;
    auto fz = formulas::fz(Vf, n, Zc);
    auto ae = y_width;
    auto ap = z_width;
    auto Q = formulas::Q(ap, ae, Vf);
    std::cerr << "Feed per tooth: " << fz << " mm\n";
    std::cerr << "Material Removal Rate: " << Q << " cm3/min\n";

    std::cerr << " len: " << length_mm(length) << " x" << x_width << " y" << y_width << " z" << z_width << "\n";
    std::cerr << "mm/rev: " << dist_per_rev << "\n";
    if(true) // testing
    {
        static int i = 0;
        std::stringstream s;
        s << "crap" << i << ".off";
        std::ofstream f(s.str());
        f << geom::format::off << mat;
        ++i;
    }
    return 0.0;
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

/* abstract out tool defs from models + add drill model where 'flutes' is tapered tip
 * */
void rs274_feedrate::tool_change(int slot) {
    using namespace machine_config;

    switch (get_machine_type(config, machine_id)) {
        case machine_type::mill: {
            mill_tool& t = _tool.mill;
            get_tool(config, slot, machine_id, t);
            auto shank = geom::make_cone( {0, 0, t.length}, {0, 0, t.flute_length}, t.shank_diameter/2, t.shank_diameter/2, 32);
            auto flutes = geom::make_cone( {0, 0, t.flute_length}, {0, 0, 0}, t.diameter/2, t.diameter/2, 32);
            _toolmodel = flutes;
            _tool_shank = shank;
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

rs274_feedrate::rs274_feedrate(boost::program_options::variables_map& vm, const std::string& stock_filename)
 : rs274_base(vm) {
    std::ifstream is(stock_filename);
    throw_if(!(is >> geom::format::off >> _model), "Unable to read stock from file");
}

