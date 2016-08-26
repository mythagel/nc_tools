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
 * rs274_transform.cpp
 *
 *  Created on: 2016-07-23
 *      Author: nicholas
 */

#include "rs274_transform.h"
#include <iostream>

void rs274_transform::_rapid(const Position&) {
}

void rs274_transform::_arc(const Position&, const Position&, const cxxcam::math::vector_3&, int) {
}


void rs274_transform::_linear(const Position&) {
}

rs274_transform::rs274_transform(boost::program_options::variables_map& vm)
 : rs274_base(vm) {
}

