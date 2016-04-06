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

#include "rs274_path.h"
#include <iostream>
#include <sstream>

#include "../r6.h"

void rs274_path::_rapid(const Position&) {
    if (!path_.empty())
        throw std::runtime_error("Rapid within profile disallowed");
}
void rs274_path::_arc(const Position& end, const Position& center, const cxxcam::math::vector_3& plane, int rotation) {
    using cxxcam::units::length_mm;

    if (plane.y != 1)
        throw std::runtime_error("Arc must exist in XZ plane");
    if (std::abs(end.y - program_pos.y) > 0)
        throw std::runtime_error("Helix not supported in path");
    if (std::abs(rotation) > 1)
        throw std::runtime_error("Single rotation only in path");

    auto a = convert(program_pos);
    auto b = convert(end);
    auto c = convert(center);
    arc_2 arc;
    arc.dir = rotation < 0 ? arc_2::cw : arc_2::ccw;
    arc.a = {length_mm(a.X).value(), length_mm(a.Z).value()};
    arc.b = {length_mm(b.X).value(), length_mm(b.Z).value()};
    arc.c = {length_mm(c.X).value(), length_mm(c.Z).value()};
    path_.push_back(arc);
}


void rs274_path::_linear(const Position& pos) {
    using cxxcam::units::length_mm;

    if (_active_plane != Plane::XZ)
        throw std::runtime_error("Path must be described in XZ plane");
    if (std::abs(pos.y - program_pos.y) > 0)
        throw std::runtime_error("Path must be 2d");

    auto a = convert(program_pos);
    auto b = convert(pos);
    line_segment_2 line;
    line.a = {length_mm(a.X).value(), length_mm(a.Z).value()};
    line.b = {length_mm(b.X).value(), length_mm(b.Z).value()};
    path_.push_back(line);
}

rs274_path::rs274_path()
 : rs274_base() {
}

std::vector<rs274_path::geometry> rs274_path::path() const {
    return path_;
}
