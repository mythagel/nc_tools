/* 
 * Copyright (C) 2013  Nicholas Gill
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
 * rs274_model.cpp
 *
 *  Created on: 2015-05-22
 *      Author: nicholas
 */

#include "rs274_model.h"
#include <cmath>
#include <cstring>
#include "Path.h"
#include "Simulation.h"
#include <fstream>
#include "throw_if.h"
#include "geom/primitives.h"

void rs274_model::_rapid(const Position&) {
}

void rs274_model::_arc(const Position& end, const Position& center, const cxxcam::math::vector_3& plane, int rotation) {
	auto steps = cxxcam::path::expand_arc(convert(program_pos), convert(end), convert(center), (rotation < 0 ? cxxcam::path::ArcDirection::Clockwise : cxxcam::path::ArcDirection::CounterClockwise), plane, std::abs(rotation), {}).path;
    _model = cxxcam::simulation::remove_material(_tool, _model, steps);
}


void rs274_model::_linear(const Position& pos) {
	auto steps = cxxcam::path::expand_linear(convert(program_pos), convert(pos), {}, -1).path;
    _model = cxxcam::simulation::remove_material(_tool, _model, steps);
}
void rs274_model::tool_change(int slot) {
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

	auto shank = geom::make_cone( {0, 0, length}, {0, 0, flute_length}, shank_diameter, shank_diameter, 8);
    auto flutes = geom::make_cone( {0, 0, flute_length}, {0, 0, 0}, diameter, diameter, 8);
    _tool = shank + flutes;

    lua_pop(L, 1);
}

rs274_model::rs274_model(const std::string& stock_filename)
 : rs274_base() {
    std::ifstream is(stock_filename);
    throw_if(!(is >> geom::format::off >> _model), "Unable to read stock from file");
}

geom::polyhedron_t rs274_model::model() const {
    return _model;
}
