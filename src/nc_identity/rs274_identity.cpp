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
 * rs274_identity.cpp
 *
 *  Created on: 2015-11-12
 *      Author: nicholas
 */

#include "rs274_identity.h"
#include <iostream>
#include <sstream>

#include "../r6.h"

void rs274_identity::_rapid(const Position&) {
}

void rs274_identity::_arc(const Position&, const Position&, const cxxcam::math::vector_3&, int) {
}


void rs274_identity::_linear(const Position&) {
}

void rs274_identity::block_end(const block_t& block) {
    std::ostringstream s;
    auto out = [&s](char A, maybe<double> a){
        if (a) s << A << r6(*a) << ' ';
    };
    auto outi = [&s](char A, maybe<unsigned int> a){
        if (a) s << A << r6(*a) << ' ';
    };
    outi('N', block.line_number);

    for (unsigned i = 0; i < 15; ++i) {
        if (block.g_modes[i] != -1)
            out('G', static_cast<double>(block.g_modes[i])/10.0);
    }

    out('X', block.x);
    out('Y', block.y);
    out('Z', block.z);
    out('A', block.a);
    out('B', block.b);
    out('C', block.c);

    outi('H', block.h);

    out('I', block.i);
    out('J', block.j);
    out('K', block.k);

    outi('L', block.l);

    out('P', block.p);
    out('Q', block.q);
    out('R', block.r);
    out('S', block.s);

    for (unsigned i = 0; i < 10; ++i) {
        if (block.m_modes[i] != -1)
            outi('M', block.m_modes[i]);
    }

    outi('T', block.t);
    out('D', block.d);
    out('F', block.f);

    if (block.comment[0]) s << "(" << block.comment << ") ";

    std::cout << s.str() << "\n";
}
rs274_identity::rs274_identity()
 : rs274_base() {
}

