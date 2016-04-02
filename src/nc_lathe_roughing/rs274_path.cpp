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
void rs274_path::_arc(const Position& end, const Position& center, const cxxcam::math::vector_3&, int) {
    // TODO handle plane
    arc_2 arc;
    arc.a = {program_pos.x, program_pos.z};
    arc.b = {end.x, end.z};
    arc.c = {center.x, center.z};
    path_.push_back(arc);
}


void rs274_path::_linear(const Position& pos) {
    // TODO verify 2d / yz plane
    line_segment_2 line;
    line.a = {program_pos.x, program_pos.z};
    line.b = {pos.x, pos.z};
    path_.push_back(line);
}

rs274_path::rs274_path()
 : rs274_base() {
}

std::vector<rs274_path::geometry> rs274_path::path() const {
    return path_;
}
