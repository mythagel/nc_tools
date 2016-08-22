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
 * rs274_delay.cpp
 *
 *  Created on: 2015-07-24
 *      Author: nicholas
 */

#include "rs274_delay.h"
#include <cmath>
#include <cstring>
#include <algorithm>
#include <chrono>
#include <thread>
#include "Path.h"
#include "../throw_if.h"

double rs274_delay::motion_duration_s(const cxxcam::units::length& motion_length) const {
    using namespace cxxcam;

    throw_if(_feed_rate == 0, "Zero feed rate");

    auto feed_rate = [&] {
        if(_length_unit_type == Units::Metric) {
            return units::velocity{ (_feed_rate*scale) * units::millimeters_per_minute };
        } else {
            return units::velocity{ (_feed_rate*scale) * units::inches_per_minute };
        }
    }();

    auto time = units::time{ motion_length / feed_rate };
    // TODO is a static assert that time is represented as seconds necessary?
    return time.value();
}

void rs274_delay::_rapid(const Position&) {
    using namespace cxxcam;

	//auto length = path::length_linear(convert(program_pos), convert(pos));
    // TODO rapid rate?
}

void rs274_delay::_arc(const Position& end, const Position& center, const cxxcam::math::vector_3& plane, int rotation) {
    using namespace cxxcam;

	auto length = path::length_arc(convert(program_pos), convert(end), convert(center), (rotation < 0 ? path::ArcDirection::Clockwise : path::ArcDirection::CounterClockwise), plane, std::abs(rotation));
    auto time_delta = std::chrono::duration<double>(motion_duration_s(length));
    duration += time_delta;
    if (!measure_only)
        std::this_thread::sleep_for(time_delta);
}


void rs274_delay::_linear(const Position& pos) {
    using namespace cxxcam;

	auto length = path::length_linear(convert(program_pos), convert(pos));
    auto time_delta = std::chrono::duration<double>(motion_duration_s(length));
    duration += time_delta;
    if (!measure_only)
        std::this_thread::sleep_for(time_delta);
}

rs274_delay::rs274_delay(boost::program_options::variables_map& vm, double scale, bool measure_only)
 : rs274_base(vm), scale(scale > 0 ? scale : 1), measure_only(measure_only) {
}

std::chrono::duration<double> rs274_delay::cut_duration() {
    return duration;
}
