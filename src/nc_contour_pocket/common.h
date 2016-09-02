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

namespace geometry {

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
point_2 lerp(const point_2& p0, const point_2& p1, double t) {
    return { (1-t)*p0.x + t*p1.x, (1-t)*p0.y + t*p1.y };
}

struct vector_2
{
    double x;
    double y;
};
inline double dot(const vector_2& a, const vector_2& b) {
    return a.x * b.x + a.y * b.y;
}

struct line_segment_2
{
    point_2 a;
    point_2 b;
};

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

template<typename T>
struct maybe {
	bool valid;
	T value;

	maybe() : valid(false) {}
	maybe(T v) : valid(true), value(v) {}

	explicit operator bool() const { return valid; }
	maybe& operator= (T v) {
		valid = true;
		value = v;
		return *this;
	}
	T operator*() const {
		if(!valid)
			throw std::logic_error("Value not valid.");
		return value;
	}
};

maybe<point_2> intersects (const line_segment_2& l1, const line_segment_2& l2) {
    auto s1 = l1.b - l1.a;
    auto s2 = l2.b - l2.a;

    vector_2 ortho{s2.y, -s2.x};
    auto denom = dot({s1.x, s1.y}, ortho);
    if (std::abs(denom) < 0.000001)
        return {};

    auto u = l1.a - l2.a;
    auto s = ( s2.x * u.y - s2.y * u.x) / denom;
    auto t = (-s1.y * u.x + s1.x * u.y) / denom;

    if (s >= 0 && s <= 1 && t >= 0 && t <= 1)
        return { l2.a + (s2 * t) };

    return {};
}

maybe<point_2> intersects (const line_segment_2& l1, const std::vector<point_2>& polygon) {
		auto it = begin(polygon);
		auto p0 = *it++;
		while (it != end(polygon)) {
				auto p1 = *it++;
				if (auto p = intersects(l1, line_segment_2{p0, p1}))
						return p;
				p0 = p1;
		}
		return {};
}

}

#endif /* COMMON_H_ */
