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
#include "base/machine_config.h"

#include <iostream>

namespace {

unsigned int hardware_concurrency() {
    auto cores = std::thread::hardware_concurrency();
    if(!cores) cores = 4;
    return cores;
}

}

/*
 * vector of future polyhedrons
 * queue of packaged_tasks
 * pool pulls tasks from queue
 * ...
 * */
geom::polyhedron_t parallel_fold_toolpath(std::vector<geom::polyhedron_t> tool_motion);
std::vector<geom::polyhedron_t> parallel_fold_toolpath(unsigned int n, std::vector<geom::polyhedron_t> tool_motion);

void rs274_model::_rapid(const Position& pos) {
    using namespace cxxcam;

	auto length = path::length_linear(convert(program_pos), convert(pos));
    auto spindle_delta = spindle_delta_theta(length);
    apply_spindle_delta(spindle_delta);
}

void rs274_model::_arc(const Position& end, const Position& center, const cxxcam::math::vector_3& plane, int rotation) {
    using namespace cxxcam;

	auto length = path::length_arc(convert(program_pos), convert(end), convert(center), (rotation < 0 ? path::ArcDirection::Clockwise : path::ArcDirection::CounterClockwise), plane, std::abs(rotation));
    auto spindle_theta = _spindle_theta;
    auto spindle_delta = spindle_delta_theta(length);
    apply_spindle_delta(spindle_delta);
    auto spindle_steps = (spindle_delta / (2*PI)) * _steps_per_revolution;
    auto spindle_step = spindle_delta / spindle_steps;

	auto steps = path::expand_arc(convert(program_pos), convert(end), convert(center), (rotation < 0 ? path::ArcDirection::Clockwise : path::ArcDirection::CounterClockwise), plane, std::abs(rotation), {}, _lathe ? spindle_steps : 1).path;

    fold_adjacent(std::begin(steps), std::end(steps), std::back_inserter(_toolpath), 
		[&](const path::step& s0, const path::step& s1) -> geom::polyhedron_t
		{
            if (_lathe) {
                auto tp = simulation::sweep_lathe_tool(_tool, s0, s1, units::plane_angle(spindle_theta * units::radians));
                spindle_theta += spindle_step;
                return tp;
            } else {
                return simulation::sweep_tool(_tool, s0, s1);
            }
		});
    if(_toolpath.size() >= 512 * hardware_concurrency())
        _toolpath = parallel_fold_toolpath(hardware_concurrency(), _toolpath);
}


void rs274_model::_linear(const Position& pos) {
    using namespace cxxcam;

	auto length = path::length_linear(convert(program_pos), convert(pos));
    auto spindle_theta = _spindle_theta;
    auto spindle_delta = spindle_delta_theta(length);
    apply_spindle_delta(spindle_delta);
    auto spindle_steps = (spindle_delta / (2*PI)) * _steps_per_revolution;
    auto spindle_step = spindle_delta / spindle_steps;

	auto steps = path::expand_linear(convert(program_pos), convert(pos), {}, _lathe ? spindle_steps : -1).path;

    fold_adjacent(std::begin(steps), std::end(steps), std::back_inserter(_toolpath), 
		[&](const path::step& s0, const path::step& s1) -> geom::polyhedron_t
		{
            if (_lathe) {
                auto tp = simulation::sweep_lathe_tool(_tool, s0, s1, units::plane_angle(spindle_theta * units::radians));
                spindle_theta += spindle_step;
                return tp;
            } else {
			    return simulation::sweep_tool(_tool, s0, s1);
            }
		});
    if(_toolpath.size() >= 512 * hardware_concurrency())
        _toolpath = parallel_fold_toolpath(hardware_concurrency(), _toolpath);
}
/* abstract out tool defs from models + add drill model where 'flutes' is tapered tip
 * */
void rs274_model::tool_change(int slot) {
    using namespace machine_config;

    if (_lathe) {
        lathe_tool t;
        get_tool(config, slot, machine_id, t);
        // TODO
    } else {
        mill_tool t;
        get_tool(config, slot, machine_id, t);
        auto shank = geom::make_cone( {0, 0, t.length}, {0, 0, t.flute_length}, t.shank_diameter/2, t.shank_diameter/2, 32);
        auto flutes = geom::make_cone( {0, 0, t.flute_length}, {0, 0, 0}, t.diameter/2, t.diameter/2, 32);
        //_tool = shank + flutes;
        _tool = flutes;
    }
}

void rs274_model::dwell(double /*seconds*/) {
    // TODO update spindle theta based on dwell time
}

rs274_model::rs274_model(boost::program_options::variables_map& vm, const std::string& stock_filename)
 : rs274_base(vm) {
    std::ifstream is(stock_filename);
    throw_if(!(is >> geom::format::off >> _model), "Unable to read stock from file");

    auto type = machine_config::get_machine_type(config, machine_id);
    _lathe = type == machine_config::machine_type::lathe;
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
geom::polyhedron_t parallel_fold_toolpath(std::vector<geom::polyhedron_t> tool_motion) {
    auto cores = hardware_concurrency();
    while(cores > 1) {
        tool_motion = parallel_fold_toolpath(cores, tool_motion);
        cores /= 2;
    }
    return geom::merge(tool_motion);
}

geom::polyhedron_t rs274_model::model() {
    if(!_toolpath.empty()) {
        auto toolpath = parallel_fold_toolpath(_toolpath);
        if (_lathe)
        std::cerr << geom::format::off << toolpath;
        _toolpath.clear();
        _model -= toolpath;
    }
    return _model;
}
