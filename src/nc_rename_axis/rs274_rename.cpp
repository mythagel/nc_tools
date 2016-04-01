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
 * rs274_rename.cpp
 *
 *  Created on: 2016-04-01
 *      Author: nicholas
 */

#include "rs274_rename.h"
#include <iostream>

void rs274_rename::_rapid(const Position&) {
}

void rs274_rename::_arc(const Position&, const Position&, const cxxcam::math::vector_3&, int) {
}


void rs274_rename::_linear(const Position&) {
}

void rs274_rename::block_end(const block_t& b) {
    using std::swap;
    block_t block = b;

    enum
    {
        G_0 = 0,
        G_1 = 10,
        G_2 = 20,
        G_3 = 30
    };

    if (
        block.motion_to_be == G_0 || 
        block.motion_to_be == G_1 || 
        block.motion_to_be == G_2 || 
        block.motion_to_be == G_3
        ) {
        // TODO swap from configuration, delete == swap with empty optional
        swap(block.x, block.a);
    }

    std::cout << str(block) << "\n";
}
rs274_rename::rs274_rename()
 : rs274_base() {
}

