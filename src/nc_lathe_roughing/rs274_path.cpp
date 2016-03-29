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
}
void rs274_path::_arc(const Position& end, const Position& center, const cxxcam::math::vector_3& plane, int rotation) {
    // TODO handle plane
    arc_2 arc;
    arc.a = {program_pos.x, program_pos.y};
    arc.b = {end.x, end.y};
    arc.c = {center.x, center.y};
}


void rs274_path::_linear(const Position& pos) {
    // TODO verify 2d / yz plane
    line_segment_2 line;
    line.a = {program_pos.x, program_pos.y};
    line.b = {pos.x, pos.y};
    path_.push_back(line);
}

rs274_path::rs274_path()
 : rs274_base() {
}

std::vector<rs274_path::geometry> rs274_path::path() const {
    return path_;
}
