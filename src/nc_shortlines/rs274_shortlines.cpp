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
 * rs274_shortlines.cpp
 *
 *  Created on: 2016-04-06
 *      Author: nicholas
 */

#include "rs274_shortlines.h"
#include <cmath>
#include <cstring>
#include <algorithm>
#include <chrono>
#include <thread>
#include "Path.h"
#include "../throw_if.h"
#include "../r6.h"
#include <iostream>
#include "base/machine_config.h"

void rs274_shortlines::_rapid(const Position& pos) {
    using namespace cxxcam::path;

	auto steps = expand_linear(convert(program_pos), convert(pos), {}, 10).path;
    for (auto& step : steps) {
        auto& p = step.position;
        output_point(p, true);
    }
}

void rs274_shortlines::_arc(const Position& end, const Position& center, const cxxcam::math::vector_3& plane, int rotation) {
    using namespace cxxcam::path;

	auto steps = expand_arc(convert(program_pos), convert(end), convert(center), (rotation < 0 ? ArcDirection::Clockwise : ArcDirection::CounterClockwise), plane, std::abs(rotation), {}, 10).path;
    for (auto& step : steps) {
        auto& p = step.position;
        output_point(p, false);
    }
}


void rs274_shortlines::_linear(const Position& pos) {
    using namespace cxxcam::path;

	auto steps = expand_linear(convert(program_pos), convert(pos), {}, 10).path;
    for (auto& step : steps) {
        auto& p = step.position;
        output_point(p, false);
    }
}

void rs274_shortlines::output_point(const cxxcam::math::point_3& p, bool rapid) const {
    using cxxcam::units::length_mm;
    using cxxcam::units::length_inch;

    if (rapid)
        std::cout << "G0";
    else
        std::cout << "G1";

    switch (machine_config::machine_units(config, machine_id)) {
        case machine_config::units::metric:
            std::cout << " X" << r6(length_mm(p.x).value());
            std::cout << " Y" << r6(length_mm(p.y).value());
            std::cout << " Z" << r6(length_mm(p.z).value());
            break;
        case machine_config::units::imperial:
            std::cout << " X" << r6(length_inch(p.x).value());
            std::cout << " Y" << r6(length_inch(p.y).value());
            std::cout << " Z" << r6(length_inch(p.z).value());
            break;
        default:
            throw std::logic_error("Unhandled units");
    }

    if (!rapid)
        std::cout << " F" << _feed_rate;
    std::cout << "\n";
}

rs274_shortlines::rs274_shortlines(boost::program_options::variables_map& vm)
 : rs274_base(vm) {
}

