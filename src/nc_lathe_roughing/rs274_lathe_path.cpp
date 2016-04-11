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
 * rs274_path.cpp
 *
 *  Created on: 2015-11-12
 *      Author: nicholas
 */

#include "rs274_lathe_path.h"
#include <iostream>
#include <sstream>
#include "Path.h"
#include "../fold_adjacent.h"

point_2 make_point(const cxxcam::math::point_3& p) {
    using cxxcam::units::length_mm;
    return {length_mm(p.x).value(), length_mm(p.z).value()};
}

void rs274_path::_rapid(const Position& p) {
    using cxxcam::units::length_mm;
    if (!path_.empty()) {
        throw std::runtime_error("Rapid within profile disallowed");
    } else {
        auto pos = convert(p);
        start_point_ = {length_mm(pos.X).value(), length_mm(pos.Z).value()};
    }
}

void rs274_path::_arc(const Position& end, const Position& center, const cxxcam::math::vector_3& plane, int rotation) {
    using cxxcam::units::length_mm;

    if (plane.y != 1)
        throw std::runtime_error("Arc must exist in XZ plane");
    if (std::abs(end.y - program_pos.y) > 0)
        throw std::runtime_error("Helix not supported in path");
    if (std::abs(rotation) > 1)
        throw std::runtime_error("Single rotation only in path");

    using namespace cxxcam::path;
	auto steps = expand_arc(convert(program_pos), convert(end), convert(center), (rotation < 0 ? ArcDirection::Clockwise : ArcDirection::CounterClockwise), plane, std::abs(rotation), {}).path;

    fold_adjacent(std::begin(steps), std::end(steps), std::back_inserter(path_),
        [this](const cxxcam::path::step& s0, const cxxcam::path::step& s1) -> line_segment_2
        {
            return { make_point(s0.position), make_point(s1.position) };
        });
}

void rs274_path::_linear(const Position& pos) {
    using cxxcam::units::length_mm;

    if (_active_plane != Plane::XZ)
        throw std::runtime_error("Path must be described in XZ plane");
    if (std::abs(pos.y - program_pos.y) > 0)
        throw std::runtime_error("Path must be 2d");

	auto steps = cxxcam::path::expand_linear(convert(program_pos), convert(pos), {}, -1).path;

    fold_adjacent(std::begin(steps), std::end(steps), std::back_inserter(path_),
        [this](const cxxcam::path::step& s0, const cxxcam::path::step& s1) -> line_segment_2
        {
            return { make_point(s0.position), make_point(s1.position) };
        });
}

rs274_path::rs274_path()
 : rs274_base() {
}

point_2 rs274_path::start_point() const {
    return start_point_;
}

std::vector<line_segment_2> rs274_path::path() const {
    return path_;
}
