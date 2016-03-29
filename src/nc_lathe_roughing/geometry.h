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
 * geometry.h
 *
 *  Created on: 2016-03-25
 *      Author: nicholas
 */

#ifndef GEOMETRY_H_
#define GEOMETRY_H_
#include <cmath>
#include <boost/optional.hpp>

struct point_2
{
    double x;
    double y;

    point_2 operator-(const point_2& p) const {
        return {x-p.x, y-p.y};
    }
    point_2 operator+(const point_2& p) const {
        return {x+p.x, y+p.y};
    }
    point_2 operator*(double t) const {
        return {x*t, y*t};
    }
};
inline double distance(const point_2& a, const point_2& b) {
    return std::sqrt(std::pow(b.x - a.x, 2) + std::pow(b.y - a.y, 2));
}

struct vector_2
{
    double x;
    double y;
};
inline double dot(const vector_2& a, const vector_2& b) {
    return a.x * b.x + a.y * b.y;
}

struct ray_2
{
    point_2 p;
    point_2 q;
};

struct line_segment_2
{
    point_2 a;
    point_2 b;
};
struct arc_2
{
    enum {
        cw,
        ccw
    } dir;
    point_2 a;
    point_2 b;
    point_2 c;
};
inline double radius(const arc_2& arc) {
    return std::abs(distance(arc.a, arc.c));
}
inline double theta(const arc_2& arc, const point_2& p) {
    auto t = atan2(p.y - arc.c.y, p.x - arc.c.x);
    if(t < 0) return (2*3.1415926) + t;
    return t;
}

boost::optional<point_2> intersects(const line_segment_2& l1, const ray_2& r);
std::pair<boost::optional<point_2>, boost::optional<point_2>> intersects(const arc_2& arc, const ray_2& ray);

#endif /* GEOMETRY_H_ */
