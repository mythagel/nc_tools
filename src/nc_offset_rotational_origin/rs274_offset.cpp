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
 * rs274_offset.cpp
 *
 *  Created on: 2015-11-12
 *      Author: nicholas
 */

#include "rs274_offset.h"
#include <iostream>
#include <iomanip>
#include <sstream>

std::string r6(double v) {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(6) << v;
    auto s = ss.str();
    
    s.erase(s.find_last_not_of('0') + 1, std::string::npos);
    if(s.back() == '.') s.pop_back();
    return s;
}

void rs274_offset::_rapid(const Position& pos) {
}

void rs274_offset::_arc(const Position&, const Position&, const cxxcam::math::vector_3&, int) {
}


void rs274_offset::_linear(const Position&) {
}

void rs274_offset::block_end() {
    char line[256];
    line_text(line, 256);
    std::cout << line << "\n";
}

rs274_offset::rs274_offset(const rotational_origin& from, const rotational_origin& to)
 : rs274_base(), from_(from), to_(to) {
}

