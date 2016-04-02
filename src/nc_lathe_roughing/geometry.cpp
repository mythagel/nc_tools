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

boost::optional<point_2> intersects(const line_segment_2& l1, const ray_2& r) {
    auto s1 = l1.b - l1.a;
    auto s2 = r.q - r.p;

    vector_2 ortho{s2.z, -s2.x};
    auto denom = dot({s1.x, s1.z}, ortho);
    if (std::abs(denom) < 0.000001)
        return {};

    auto u = l1.a - r.p;
    auto s = ( s2.x * u.z - s2.z * u.x) / denom;
    auto t = (-s1.z * u.x + s1.x * u.z) / denom;

    if (s >= 0 && t >= 0 && t <= 1)
        return { r.p + (s2 * t) };

    return {};
}

std::pair<boost::optional<point_2>, boost::optional<point_2>> intersects(const arc_2& arc, const ray_2& ray) {
    std::pair<boost::optional<point_2>, boost::optional<point_2>> intersections;

    auto d = ray.q - ray.p;
    auto pc = ray.p - arc.c;

    auto pi2 = 2.0 * 3.14159265359;
    auto start_theta = theta(arc, arc.a);
    auto end_theta = theta(arc, arc.b);
    auto delta_theta = end_theta - start_theta;
    switch(arc.dir) {
        case arc_2::cw:
            if (delta_theta > 0)
                delta_theta -= pi2;
            else if (delta_theta == 0)
                delta_theta = -pi2;
            break;
        case arc_2::ccw:
            if (delta_theta < 0)
                delta_theta += pi2;
            else if (delta_theta == 0)
                delta_theta = pi2;
            break;
    }

    auto A = std::pow(d.x, 2) + std::pow(d.z, 2);
    auto B = 2 * (d.x * pc.x + d.z * pc.z);
    auto C = std::pow(pc.x, 2) + std::pow(pc.z, 2) - std::pow(radius(arc), 2);

    auto contains = [&](double a) -> bool {
        auto a1 = a;
        if (a == 0.0) a1 = pi2;
        auto start = start_theta;
        auto end = start_theta + delta_theta;
        if (end > start)
            return (a >= start && a <= end) || (a1 >= start && a1 <= end);
        else
            return (a >= end && a <= start) || (a1 >= end && a1 <= start);
    };

    auto disc = std::pow(B, 2) - (4*A*C);
    if ((std::abs(A) < 0.000001) || disc < 0)
    {
        // no intersection
    }
    else if (disc == 0)
    {
        // Tangent
        auto t1 = -B / (2 * A);
        point_2 p1 = {ray.p.x + t1 * d.x, ray.p.z + t1 * d.z};
        auto theta1 = theta(arc, p1);
        if(contains(theta1))
            intersections.first = p1;
    }
    else
    {
        // Intersection
        auto t1 = (-B + std::sqrt(disc)) / (2 * A);
        auto t2 = (-B - std::sqrt(disc)) / (2 * A);
        point_2 p1 = {ray.p.x + t1 * d.x, ray.p.z + t1 * d.z};
        point_2 p2 = {ray.p.x + t2 * d.x, ray.p.z + t2 * d.z};
        auto theta1 = theta(arc, p1);
        auto theta2 = theta(arc, p2);
        if(contains(theta1))
            intersections.first = p1;
        if(contains(theta2))
            intersections.second = p2;
    }
    return intersections;
}

boost::optional<point_2> intersects (const line_segment_2& l1, const line_segment_2& l2) {
    auto s1 = l1.b - l1.a;
    auto s2 = l2.b - l2.a;

    vector_2 ortho{s2.z, -s2.x};
    auto denom = dot({s1.x, s1.z}, ortho);
    if (denom > -0.000001 && denom < 0.000001)
        return {};

    auto u = l1.a - l2.a;
    auto s = ( s2.x * u.z - s2.z * u.x) / denom;
    auto t = (-s1.z * u.x + s1.x * u.z) / denom;

    if (s >= 0 && s <= 1 && t >= 0 && t <= 1)
        return { l1.a + (s1 * t) };

    return {};
}
