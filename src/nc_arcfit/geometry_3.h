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
 * geometry_3.h
 *
 *  Created on: 2016-04-11
 *      Author: nicholas
 */

#ifndef GEOMETRY_3_H_
#define GEOMETRY_3_H_
#include <boost/optional.hpp>
#include <iostream>
#include <cmath>

namespace geometry_3 {

struct point_3;
struct vector_3;

struct point_3 {
    double x;
    double y;
    double z;

    vector_3 operator-(const point_3& p) const;
};

struct line_3 {
    point_3 a;
    point_3 b;
};

struct vector_3 {
    double x;
    double y;
    double z;

    vector_3 operator-(const vector_3& v) const;
    vector_3 operator+(const vector_3& v) const;
};

double distance(const point_3& p0, const point_3& p1);

// NOTE: points must represent an ordered polyline
bool collinear(const point_3& a, const point_3& b, const point_3& c, double tolerance = 1e-9);

vector_3 cross(const vector_3& a, const vector_3& b);
vector_3 normalise(const vector_3& v);

vector_3 plane(const point_3& a, const point_3& b, const point_3& c);

double chord_height(const point_3& a, const point_3& b, double r);

boost::optional<point_3> circle_center(point_3 p0, point_3 p1, point_3 p2);

inline std::ostream& operator<<(std::ostream& os, const point_3& p) {
    os << "{" << p.x << "," << p.y << "," << p.z << "}";
    return os;
}

inline double theta(const geometry_3::point_3& p, const geometry_3::point_3& center) {
    auto t = std::atan((p.y - center.y) / (p.x - center.x));
    if (t < 0) t += 6.28318530718;
    return t;
};

}

#endif /* GEOMETRY_3_H_ */
