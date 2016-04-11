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
    double z;

    point_2 operator-(const point_2& p) const {
        return {x-p.x, z-p.z};
    }
    point_2 operator+(const point_2& p) const {
        return {x+p.x, z+p.z};
    }
    point_2 operator*(double t) const {
        return {x*t, z*t};
    }
};
inline double distance(const point_2& a, const point_2& b) {
    return std::sqrt(std::pow(b.x - a.x, 2) + std::pow(b.z - a.z, 2));
}

struct vector_2
{
    double x;
    double z;
};
inline double dot(const vector_2& a, const vector_2& b) {
    return a.x * b.x + a.z * b.z;
}

struct line_segment_2
{
    point_2 a;
    point_2 b;
};

boost::optional<point_2> intersects(const line_segment_2& l1, const line_segment_2& l2);

#endif /* GEOMETRY_H_ */
