/* Copyright (C) 2016  Nicholas Gill
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
 * 
 * common.h
 *
 *  Created on: 2016-08-29
 *      Author: nicholas
 */

#ifndef COMMON_H_
#define COMMON_H_
#include <cmath>
#include <vector>

struct point_2
{
    double x;
    double y;
};
inline double distance(const point_2& a, const point_2& b) {
    return std::sqrt(std::pow(b.x - a.x, 2) + std::pow(b.y - a.y, 2));
}

inline point_2 centroid(const std::vector<point_2>& polygon) {
    point_2 c = {0, 0};
    double A = 0;
    auto it = begin(polygon);
    for (unsigned i = 0; i < polygon.size(); ++i) {
        auto p = *it++;
        if (it == end(polygon))
            it = begin(polygon);
        auto p1 = *it;
        c.x += (p.x + p1.x) * ((p.x * p1.y) - (p1.x * p.y));
        c.y += (p.y + p1.y) * ((p.x * p1.y) - (p1.x * p.y));
        A += ((p.x * p1.y) - (p1.x * p.y));
    }

    A /= 2.0;
    c.x /= 6*A;
    c.y /= 6*A;
    return c;
}

#endif /* COMMON_H_ */
