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
 * rs274_arcfit.cpp
 *
 *  Created on: 2016-04-11
 *      Author: nicholas
 */

#include "rs274_arcfit.h"
#include <iostream>
#include "Units.h"

void rs274_arcfit::_rapid(const Position&) {
}

void rs274_arcfit::_arc(const Position&, const Position&, const cxxcam::math::vector_3&, int) {
}


void rs274_arcfit::_linear(const Position&) {
}

void rs274_arcfit::block_end(const block_t& block) {
    enum {
        G_1 = 10
    };
    auto g1_only = [&](const block_t& block) {
        for (unsigned i = 0; i < 15; ++i) {
            if (block.g_modes[i] != -1 && block.g_modes[i] != G_1)
                return false;
        }
        return true;
    };
    auto no_m_modes = [&](const block_t& block) {
        for (unsigned i = 0; i < 10; ++i) {
            if (block.m_modes[i] != -1)
                return false;
        }
        return true;
    };
    auto is_linear = [&](const block_t& block) {
        return block.motion_to_be == G_1 &&         // Linear move
            !block.a && !block.b && !block.c &&     // No rotary motion
            (block.x || block.y || block.z) &&      // At least one axis word
            g1_only(block) && no_m_modes(block);    // No other G or M codes
    };
    
    /* TODO problem - blocks do not describe complete
     * content of path - current point and complete point information
     * requires maintaining state - but that state may not represent
     * the content of the block directly (tool offsets)
     *
     * However the reverse is also true, applying the arc folding to
     * the path which has already been transformed is not ideal either.
     * perhaps add an option to encode the complete current xyz position
     * into the gcode parser.
     *
     * have to then make assumptions that units are the same, etc.
     * reasonable since any other gcodes will interrupt the processing.
     *
     * really only need to reliably determine start point of candidate arc
     * to be able to fill in detail missing from blocks.
     *
     * ORRRR maybe just wait until the appopriate context has been gathered...
     * not optimal - not bad either - arc will vary across both axes in plane
     * */
    if (is_linear(block)) {
        candidate.push_back(block);
        if (candidate.size() >= 3) {
        }
    } else {
        flush(-1);
        std::cout << str(block) << "\n";
    }
    // TODO determine if block represents pure linear motion
    // if true, do arcfit processing
    // if not, flush arcfit then output.
/*    using cxxcam::units::length_mm;
    // move arcfit state machine
    auto p = convert(pos);
    points.push_back({length_mm(p.X).value(), length_mm(p.Y).value(), length_mm(p.Z).value()});

    if(points.size() > 3) {
        if (geometry_3::collinear(points[0], points[1], points[2])) {
            flush(1);
        } else {
            auto x = geometry_3::plane(points[0], points[1], points[2]);
            if (x.z != 1.0) {
                flush(1);
            } else {
                // TODO
            }
        }
    }
*/
}

void rs274_arcfit::flush(int n) {
    while (n-- && !candidate.empty()) {
        auto& block = candidate[0];
        std::cout << str(block) << "\n";
        candidate.erase(begin(candidate));
    }
}

rs274_arcfit::rs274_arcfit()
 : rs274_base() {
}

