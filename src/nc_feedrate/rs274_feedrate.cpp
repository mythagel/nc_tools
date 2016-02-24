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
#include <sstream>
#include <iomanip>
std::string r6(double v) {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(6) << v;
    auto s = ss.str();
    
    s.erase(s.find_last_not_of('0') + 1, std::string::npos);
    if(s.back() == '.') s.pop_back();
    return s;
}

std::ostream& operator<<(std::ostream& os, const geom::query::bbox_3& b) {
    os << "min: {" << r6(b.min.x) << ", " << r6(b.min.y) << ", " << r6(b.min.z) <<"} max: {" << r6(b.max.x) << ", " << r6(b.max.y) << ", " << r6(b.max.z) <<"}";
    return os;
}

void rs274_feedrate::_rapid(const Position& pos) {
    using namespace cxxcam;
	auto steps = path::expand_linear(convert(program_pos), convert(pos), {}, -1).path;

    std::vector<bool> intersections;
    fold_adjacent(std::begin(steps), std::end(steps), std::back_inserter(intersections), 
		[this](const path::step& s0, const path::step& s1) -> bool
		{
			auto toolpath = simulation::sweep_tool(_tool + _tool_shank, s0, s1);
            return intersects(toolpath, _model);
		});
    if (std::find(begin(intersections), end(intersections), true) != end(intersections)) {
        std::cerr << "rapid intersection\n";
    }
}

double chip_load(const cxxcam::path::step& s0, const cxxcam::path::step& s1, const geom::polyhedron_t& tool, geom::polyhedron_t& model) {
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
    auto tool_path = simulation::sweep_tool(tool, s0, s1);
    auto mat = model * tool_path;
    model -= tool_path;
    auto deorient = identity;
    deorient /= o0;

    auto dir = math::vector_3{length_mm(p1.x - p0.x).value(), length_mm(p1.y - p0.y).value(), length_mm(p1.z - p0.z).value()};
    // ignore z component of direction
    dir.z = 0;
    auto reorient = identity;
    reorient /= to_quat(normalise(dir), {1, 0, 0});

    mat = translate(mat, length_mm(-p0.x).value(), length_mm(-p0.y).value(), length_mm(-p0.z).value());
    mat = rotate(mat, deorient.R_component_1(), deorient.R_component_2(), deorient.R_component_3(), deorient.R_component_4());
    mat = rotate(mat, reorient.R_component_1(), reorient.R_component_2(), reorient.R_component_3(), reorient.R_component_4());
    // TODO reorient / translate before bbox
    auto bbox = bounding_box(mat);
    std::cerr << bbox << "\n";
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
	auto steps = path::expand_arc(convert(program_pos), convert(end), convert(center), (rotation < 0 ? path::ArcDirection::Clockwise : path::ArcDirection::CounterClockwise), plane, std::abs(rotation), {}, 1).path;

    std::vector<double> chip_load_per_tooth;
    fold_adjacent(std::begin(steps), std::end(steps), std::back_inserter(chip_load_per_tooth), 
		[this](const path::step& s0, const path::step& s1) -> double
		{
            return chip_load(s0, s1, _tool, _model);
		});
    for(auto& clpt : chip_load_per_tooth) {
    }
    // TODO analyse each toolpath step to determine appropriate feed rate
}


void rs274_feedrate::_linear(const Position& pos) {
    using namespace cxxcam;
	auto steps = path::expand_linear(convert(program_pos), convert(pos), {}, 1).path;

    std::vector<double> chip_load_per_tooth;
    fold_adjacent(std::begin(steps), std::end(steps), std::back_inserter(chip_load_per_tooth), 
		[this](const path::step& s0, const path::step& s1) -> double
		{
            return chip_load(s0, s1, _tool, _model);
		});
}

/* abstract out tool defs from models + add drill model where 'flutes' is tapered tip
 * */
void rs274_feedrate::tool_change(int slot) {
    auto& L = config.state();

    lua_getglobal(L, "tool_table");
    if (!lua_istable(L, -1)) {
        lua_pop(L, 1);
        _tool = {};
        return;
    }

    lua_pushinteger(L, slot);
    lua_gettable(L, -2);
    if (!lua_istable(L, -1)) {
        lua_pop(L, 2);
        _tool = {};
        return;
    }
    
    double length = 0.0;
    double diameter = 0.0;
    double flute_length = 0.0;
    double shank_diameter = 0.0;

    lua_getfield(L, -1, "length");
    if(lua_isnumber(L, -1))
        length = lua_tonumber(L, -1);
    lua_pop(L, 1);

    lua_getfield(L, -1, "diameter");
    if(lua_isnumber(L, -1))
        diameter = lua_tonumber(L, -1);
    lua_pop(L, 1);

    lua_getfield(L, -1, "flute_length");
    if(lua_isnumber(L, -1))
        flute_length = lua_tonumber(L, -1);
    lua_pop(L, 1);

    lua_getfield(L, -1, "shank_diameter");
    if(lua_isnumber(L, -1))
        shank_diameter = lua_tonumber(L, -1);
    lua_pop(L, 1);

	auto shank = geom::make_cone( {0, 0, length}, {0, 0, flute_length}, shank_diameter/2, shank_diameter/2, 32);
    auto flutes = geom::make_cone( {0, 0, flute_length}, {0, 0, 0}, diameter/2, diameter/2, 32);

    _tool = flutes;
    _tool_shank = shank;

    lua_pop(L, 1);
}

rs274_feedrate::rs274_feedrate(const std::string& stock_filename)
 : rs274_base() {
    std::ifstream is(stock_filename);
    throw_if(!(is >> geom::format::off >> _model), "Unable to read stock from file");
}

