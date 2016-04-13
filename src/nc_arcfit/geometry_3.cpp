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
 * geometry_3.cpp
 *
 *  Created on: 2016-04-11
 *      Author: nicholas
 */

#include "geometry_3.h"
#include <cmath>

namespace geometry_3 {

vector_3 point_3::operator-(const point_3& p) const {
    return {x-p.x, y-p.y, z-p.z};
}

double distance(const point_3& p0, const point_3& p1) {
    return std::sqrt((p0.x-p1.x)*(p0.x-p1.x) + (p0.y-p1.y)*(p0.y-p1.y) + (p0.z-p1.z)*(p0.z-p1.z));
}

vector_3 vector_3::operator-(const vector_3& v) const {
    return {x - v.x, y - v.y, z - v.z};
}

vector_3 vector_3::operator+(const vector_3& v) const {
    return {x - v.x, y - v.y, z - v.z};
}

vector_3 cross(const vector_3& a, const vector_3& b) {
    return {(a.y*b.z) - (a.z*b.y), (a.z*b.x) - (a.x*b.z), (a.x*b.y) - (a.y*b.x)};
}

vector_3 normalise(const vector_3& v) {
    auto scale = std::sqrt((v.x*v.x) + (v.y*v.y) + (v.z*v.z));
    if(scale == 0.0)
        return {0, 0, 0};

    return {v.x/scale, v.y/scale, v.z/scale};
}

bool collinear(const point_3& a, const point_3& b, const point_3& c, double tolerance) {
    auto AB = std::abs(distance(a, b));
    auto AC = std::abs(distance(a, c));
    auto BC = std::abs(distance(b, c));
    return std::abs((AB+BC) - AC) < tolerance;
}

vector_3 plane(const point_3& a, const point_3& b, const point_3& c) {
	auto ab = b-a;
	auto ac = c-a;
	return normalise(cross(ab, ac));
}

double chord_height(const point_3& a, const point_3& b, double r) {
    auto l = std::abs(distance(a, b)) / 2.0;
    return r - std::sqrt((r*r) - (l*l));
}

/* assumes arc is described in the xy plane; when calculating other planes, map to XY
 * */
boost::optional<point_3> circle_center(point_3 p0, point_3 p1, point_3 p2) {
    if (collinear(p0, p1, p2)) return {};

    auto perpendicular = [](const point_3& p0, const point_3& p1, double tolerance = 1e-9) {
        auto a = p1 - p0;
        return std::abs(a.x) < tolerance || std::abs(a.y) < tolerance;
    };

    if(perpendicular(p0, p1))
        std::swap(p1, p2);

    auto a = p1 - p0;
    auto b = p2 - p1;

    // check if a.x == 0 or b.x == 0
    auto ma = a.y/a.x;
    auto mb = b.y/b.x;

    auto x = (ma*mb*(p0.y - p2.y) + ma*(p0.x + p1.x) - ma*(p1.x + p2.x)) / (2*(mb-ma));
    auto ya = (-1.0/mb) * (x - ((p0.x+p1.x)/2.0)) + ((p1.y + p2.y)/2.0);

    return { {x, ya, 0} };
}

}

