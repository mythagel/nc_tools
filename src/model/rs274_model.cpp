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
#include "fold_adjacent.h"
#include "geom/ops.h"
#include <thread>
#include <future>
#include <iterator>
#include <algorithm>

namespace {

unsigned int hardware_concurrency() {
    auto cores = std::thread::hardware_concurrency();
    if(!cores) cores = 4;
    return cores;
}
}
std::vector<geom::polyhedron_t> parallel_fold_toolpath(unsigned int n, std::vector<geom::polyhedron_t> tool_motion);

void rs274_model::_rapid(const Position&) {
}

void rs274_model::_arc(const Position& end, const Position& center, const cxxcam::math::vector_3& plane, int rotation) {
    using namespace cxxcam;
	auto steps = path::expand_arc(convert(program_pos), convert(end), convert(center), (rotation < 0 ? path::ArcDirection::Clockwise : path::ArcDirection::CounterClockwise), plane, std::abs(rotation), {}).path;
    fold_adjacent(std::begin(steps), std::end(steps), std::back_inserter(_tool_motion), 
		[this](const path::step& s0, const path::step& s1) -> geom::polyhedron_t
		{
			return simulation::sweep_tool(_tool, s0, s1);
		});
    if(_tool_motion.size() > 64)
        _tool_motion = parallel_fold_toolpath(hardware_concurrency(), _tool_motion);
//    _model = simulation::remove_material(_tool, _model, steps);
}


void rs274_model::_linear(const Position& pos) {
    using namespace cxxcam;
	auto steps = path::expand_linear(convert(program_pos), convert(pos), {}, -1).path;
    fold_adjacent(std::begin(steps), std::end(steps), std::back_inserter(_tool_motion), 
		[this](const path::step& s0, const path::step& s1) -> geom::polyhedron_t
		{
			return simulation::sweep_tool(_tool, s0, s1);
		});
    if(_tool_motion.size() > 64)
        _tool_motion = parallel_fold_toolpath(hardware_concurrency(), _tool_motion);
//    _model = simulation::remove_material(_tool, _model, steps);
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

std::vector<geom::polyhedron_t> parallel_fold_toolpath(unsigned int n, std::vector<geom::polyhedron_t> tool_motion) {
    if(n == 1) return { geom::merge(tool_motion) };
    if(tool_motion.size() == 1) return tool_motion;

    unsigned int chunk_size = std::floor(tool_motion.size() / static_cast<double>(n));
    unsigned int rem = tool_motion.size() % n;
    
    typedef std::future<geom::polyhedron_t> polyhedron_future;
    std::vector<polyhedron_future> folded;

    for(unsigned int i = 0; i < n; ++i) {

        std::packaged_task<geom::polyhedron_t()> fold([&tool_motion, chunk_size, rem, i]() {
            auto begin = (chunk_size * i) + (i < rem ? i : rem);
            auto end = (begin + chunk_size) + (i < rem ? 1 : 0);

            return geom::merge(std::vector<geom::polyhedron_t>(std::make_move_iterator(tool_motion.begin() + begin), std::make_move_iterator(tool_motion.begin() + end)));
        });

        folded.push_back(fold.get_future());
        std::thread(std::move(fold)).detach();
    }

    std::vector<geom::polyhedron_t> result;
    std::transform(begin(folded), end(folded), std::back_inserter(result), [](polyhedron_future& f){ return f.get(); });
    return result;
}

geom::polyhedron_t rs274_model::model() {

    auto cores = hardware_concurrency();
    while(true) {
        _tool_motion = parallel_fold_toolpath(cores, _tool_motion);
        if (cores == 1) break;
        cores /= 2;
    }

    auto tool_path = geom::merge(_tool_motion);
    _tool_motion.clear();
    _model -= tool_path;
    return _model;
}
