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
#include <stdexcept>

AxisModification::Axis AxisModification::map(char c) {
    switch(c) {
        case 'x':
        case 'X':
            return axis_X;
        case 'y':
        case 'Y':
            return axis_Y;
        case 'z':
        case 'Z':
            return axis_Z;
        case 'a':
        case 'A':
            return axis_A;
        case 'b':
        case 'B':
            return axis_B;
        case 'c':
        case 'C':
            return axis_C;
        case 'i':
        case 'I':
            return axis_I;
        case 'j':
        case 'J':
            return axis_J;
        case 'k':
        case 'K':
            return axis_K;
        case '-':
            return axis_None;
    }
    throw std::runtime_error("Unrecognised axis");
}

void apply_mod(AxisModification a, block_t& block) {
    maybe<double> empty;
    auto axis = [&block, &empty](AxisModification::Axis a) -> maybe<double>& {
        switch (a) {
            case AxisModification::axis_None:
                return empty;

            case AxisModification::axis_X:
                return block.x;
            case AxisModification::axis_Y:
                return block.y;
            case AxisModification::axis_Z:
                return block.z;

            case AxisModification::axis_A:
                return block.a;
            case AxisModification::axis_B:
                return block.b;
            case AxisModification::axis_C:
                return block.c;

            case AxisModification::axis_I:
                return block.i;
            case AxisModification::axis_J:
                return block.j;
            case AxisModification::axis_K:
                return block.k;
        }
        throw std::logic_error("Unrecognised axis");
    };

    using std::swap;
    swap(axis(a.from), axis(a.to));
}
void rs274_rename::apply_mods(block_t& block) const {
    for (auto& mod : mods)
        apply_mod(mod, block);
}

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
    auto is_motion = [](int motion_to_be){
        return  motion_to_be == G_0 || 
                motion_to_be == G_1 || 
                motion_to_be == G_2 || 
                motion_to_be == G_3;
    };

    if (is_motion(block.motion_to_be))
        apply_mods(block);

    // TODO validate block is still valid after mods - e.g. no axis words with movement
    // must maintain non-axis words, e.g. feedrate

    std::cout << str(block) << "\n";
}
rs274_rename::rs274_rename(boost::program_options::variables_map& vm, const std::vector<AxisModification>& mods)
 : rs274_base(vm), mods(mods) {
}

