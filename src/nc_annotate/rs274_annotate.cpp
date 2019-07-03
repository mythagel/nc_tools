/* 
 * Copyright (C) 2019  Nicholas Gill
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
 * rs274_annotate.cpp
 *
 *  Created on: 2019-05-15
 *	  Author: nicholas
 */

#include "rs274_annotate.h"
#include <cmath>
#include <cstring>
#include "cxxcam/Path.h"
#include "../r6.h"
#include <iostream>
#include <sstream>
#include "circle3d.hpp"

void rs274_annotate::_rapid(const Position& pos) {
	using namespace cxxcam::path;

	auto steps = expand_linear(convert(program_pos), convert(pos), {}, 0).path;
	for (auto& step : steps) {
		auto& p = step.position;
		process_point(p, true);
	}
}

void rs274_annotate::_arc(const Position& end, const Position& center, const cxxcam::math::vector_3& plane, int rotation) {
	using namespace cxxcam::path;

	auto steps = expand_arc(convert(program_pos), convert(end), convert(center), (rotation < 0 ? ArcDirection::Clockwise : ArcDirection::CounterClockwise), plane, std::abs(rotation), {}, 1).path;
	for (auto& step : steps) {
		auto& p = step.position;
		process_point(p, false);
	}
}

void rs274_annotate::_linear(const Position& pos) {
	using namespace cxxcam::path;

	auto steps = expand_linear(convert(program_pos), convert(pos), {}, 0).path;
	for (auto& step : steps) {
		auto& p = step.position;
		process_point(p, false);
	}
}

void rs274_annotate::block_end(const block_t& block) {
	std::cout << str(block) << "\n";
	for (auto& anno : annotations_)
		std::cout << "  (" << anno << ")\n";

	annotations_.clear();
}

void rs274_annotate::process_point(const cxxcam::math::point_3& p, bool rapid) {
    using cxxcam::units::length_mm;

    if (rapid) {
        // TODO flush annotations?
        points.clear();
        return;
    }

    points.push_back(p);

    if (points.size() >= 3) {
        
        if (points.size() > 3)
            points.erase(begin(points));

        circle3d::Point3D p1 = {length_mm(points[0].x).value(), length_mm(points[0].y).value(), length_mm(points[0].z).value()};
        circle3d::Point3D p2 = {length_mm(points[1].x).value(), length_mm(points[1].y).value(), length_mm(points[1].z).value()};
        circle3d::Point3D p3 = {length_mm(points[2].x).value(), length_mm(points[2].y).value(), length_mm(points[2].z).value()};
        circle3d::Point3D c;
        double radius;
        circle3d::estimate3DCircle(p1, p2, p3, c, radius);

        double curve = 1/radius;

        if (isnan(curve) == false) {
            std::ostringstream ss;
            ss << "R" << r6(curve);     // Radius of curvature
            annotations_.push_back(ss.str());
        }
    }
}

rs274_annotate::rs274_annotate(boost::program_options::variables_map& vm)
 : rs274_base(vm) {
}

