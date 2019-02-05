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
 * rs274_path.cpp
 *
 *  Created on: 2015-11-12
 *      Author: nicholas
 */

#include "rs274_lathe_path.h"
#include "cxxcam/Path.h"
#include "base/machine_config.h"

using namespace ClipperLib;

void rs274_path::_rapid(const Position&) {
    if (!path_.empty()) {
        throw std::runtime_error("Rapid within profile disallowed");
    } else {
        using cxxcam::units::length_mm;
        using cxxcam::units::length_inch;

        auto p = convert(program_pos);
        switch (machine_config::machine_units(config, machine_id)) {
            case machine_config::units::metric:
                start_point_ = IntPoint(length_mm(p.X).value() * scale(), length_mm(p.Z).value() * scale());
                break;
            case machine_config::units::imperial:
                start_point_ = IntPoint(length_inch(p.X).value() * scale(), length_inch(p.Z).value() * scale());
                break;
            default:
                throw std::logic_error("Unhandled units");
        }
    }
}

void rs274_path::_arc(const Position& end, const Position& center, const cxxcam::math::vector_3& plane, int rotation) {
    if (plane.y != 1)
        throw std::runtime_error("Arc must exist in XZ plane");
    if (std::abs(end.y - program_pos.y) > 0)
        throw std::runtime_error("Helix not supported in path");
    if (std::abs(rotation) > 1)
        throw std::runtime_error("Single rotation only in path");

    using namespace cxxcam::path;
    auto steps = expand_arc(convert(program_pos), convert(end), convert(center), (rotation < 0 ? ArcDirection::Clockwise : ArcDirection::CounterClockwise), plane, std::abs(rotation), {}).path;

    for (auto& step : steps) {
        auto& p = step.position;
        path_.push_back(scale_point(p));
    }
}

void rs274_path::_linear(const Position& pos) {
    if (_active_plane != Plane::XZ)
        throw std::runtime_error("Path must be described in XZ plane");
    if (std::abs(pos.y - program_pos.y) > 0)
        throw std::runtime_error("Path must be 2d");

    auto steps = cxxcam::path::expand_linear(convert(program_pos), convert(pos), {}, -1).path;

    for (auto& step : steps) {
        auto& p = step.position;
        path_.push_back(scale_point(p));
    }
}

rs274_path::rs274_path(boost::program_options::variables_map& vm)
 : rs274_base(vm) {
}

IntPoint rs274_path::scale_point(const cxxcam::math::point_3& p) const {
    using cxxcam::units::length_mm;
    using cxxcam::units::length_inch;

    switch (machine_config::machine_units(config, machine_id)) {
        case machine_config::units::metric:
            return IntPoint(length_mm(p.x).value() * scale(), length_mm(p.z).value() * scale());
        case machine_config::units::imperial:
            return IntPoint(length_inch(p.x).value() * scale(), length_inch(p.z).value() * scale());
        default:
            throw std::logic_error("Unhandled units");
    }
}

Path rs274_path::path() const {
    return path_;
}
