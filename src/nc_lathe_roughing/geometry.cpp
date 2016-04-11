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
 * geometry.cpp
 *
 *  Created on: 2016-03-25
 *      Author: nicholas
 */

#include "geometry.h"

boost::optional<point_2> intersects (const line_segment_2& l1, const line_segment_2& l2) {
    auto s1 = l1.b - l1.a;
    auto s2 = l2.b - l2.a;

    vector_2 ortho{s2.z, -s2.x};
    auto denom = dot({s1.x, s1.z}, ortho);
    if (std::abs(denom) < 0.000001)
        return {};

    auto u = l1.a - l2.a;
    auto s = ( s2.x * u.z - s2.z * u.x) / denom;
    auto t = (-s1.z * u.x + s1.x * u.z) / denom;

    if (s >= 0 && s <= 1 && t >= 0 && t <= 1)
        return { l1.a + (s1 * t) };

    return {};
}
