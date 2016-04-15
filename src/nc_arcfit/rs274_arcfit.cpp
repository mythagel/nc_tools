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
 * rs274_arcfit.cpp
 *
 *  Created on: 2016-04-11
 *      Author: nicholas
 */

#include "rs274_arcfit.h"
#include <iostream>
#include "Units.h"
#include <boost/math/special_functions/sign.hpp>

namespace {
geometry_3::point_3 to_point_3(const cxxcam::Position& pos) {
    using cxxcam::units::length_mm;
    return {length_mm(pos.X).value(), length_mm(pos.Y).value(), length_mm(pos.Z).value()};
}
}

void rs274_arcfit::_rapid(const Position&) {
    point = boost::none;
}

void rs274_arcfit::_arc(const Position&, const Position&, const cxxcam::math::vector_3&, int) {
    point = boost::none;
}


void rs274_arcfit::_linear(const Position& pos) {
    // record motion point here
    point = block_point();
    point->p = to_point_3(convert(pos));
}

void rs274_arcfit::block_end(const block_t& block) {
    // but process it here iff the block describes simple linear motion
    enum {
        G_1 = 10
    };
    auto g1_only = [&](const block_t& block) {
        for (unsigned i = 0; i < 15; ++i) {
            if (block.g_modes[i] != -1 && block.g_modes[i] != G_1)
                return false;
        }
        return true;
    };
    auto no_m_modes = [&](const block_t& block) {
        for (unsigned i = 0; i < 10; ++i) {
            if (block.m_modes[i] != -1)
                return false;
        }
        return true;
    };
    auto is_linear = [&](const block_t& block) {
        return block.motion_to_be == G_1 &&         // Linear move
            !block.a && !block.b && !block.c &&     // No rotary motion
            (block.x || block.y || block.z) &&      // At least one axis word
            g1_only(block) && no_m_modes(block);    // No other G or M codes
    };

    
    if (is_linear(block) && point) {
        point->block = block;
        push(*point);
    } else {
        flush(true);
        std::cout << str(block) << "\n";
    }
    point = boost::none;
}

void rs274_arcfit::reset() {
    state = State::indeterminate;
}
void rs274_arcfit::push(const block_point& point) {
    switch (state)
    {
        case State::indeterminate:
        {
            arc.points.push_back(point);

            if (arc.points.size() < 3)
                return;
std::cout << "n " << __LINE__ << "\n";
            auto p0 = arc.points[0].p;
            auto p1 = arc.points[1].p;
            auto p2 = arc.points[2].p;

            if (collinear(p0, p1, p2, 1e-6))
                return flush();

std::cout << "n " << __LINE__ << "\n";
            // TODO handle XZ and YZ planes
            // remembering to map to xy when calculating circle center!
            arc.plane = geometry_3::plane(p0, p1, p2);
            if (arc.plane.z != 1.0)
                return flush();

std::cout << "n " << __LINE__ << "\n";
            auto center = circle_center(p0, p1, p2);
            if (!center)
                return flush();
            arc.center = *center;

std::cout << "n " << __LINE__ << "\n";
            arc.r = std::abs(distance(p0, arc.center));
            std::cout << "radius: " << arc.r << "\n";
            std::cout << "ch: " << chord_height(p0, p1, arc.r) << "\n";

            if (chord_height(p0, p1, arc.r) > arc.max_deviation)
                return flush();
std::cout << "n " << __LINE__ << "\n";
            if (chord_height(p1, p2, arc.r) > arc.max_deviation)
                return flush();
std::cout << "n " << __LINE__ << "\n";

            // need to identify direction, and initial theta
            auto t0 = std::atan((p0.y - arc.center.y) / (p0.x - arc.center.x));
            auto t1 = std::atan((p1.y - arc.center.y) / (p1.x - arc.center.x));
            std::cout << "t0: " << t0 << " t1 " << t1 << "\n";

            arc.dir = boost::math::sign(t0 - t1);
            std::cout << "dir: " << arc.dir << "\n";

            state = State::collecting_points;
            break;
        }
        case State::collecting_points:
        {
            auto pn = arc.points.back().p;

            auto r = std::abs(distance(pn, arc.center));
            std::cout << "r: " << r << " arc.r " << arc.r << "\n";
            if (std::abs(arc.r - r) > arc.point_deviation)
                flush();
            arc.points.push_back(point);

            break;
        }
    }
}
void rs274_arcfit::flush(bool all) {
    switch (state)
    {
        case State::indeterminate:
        {
            while (!arc.points.empty()) {
                auto& block = arc.points[0].block;
                std::cout << str(block) << "\n";
                arc.points.erase(begin(arc.points));

                if (!all) break;
            }
            break;
        }
        case State::collecting_points:
        {
            if (arc.points.size() > 3) {
                auto p0 = arc.points[0].p;
                auto p1 = arc.points[arc.points.size()-1].p;

                std::cout << "arc p0 " << p0.x << " " << p0.y << "\n";
                std::cout << "    p1 " << p1.x << " " << p1.y << "\n";
                std::cout << "    pc " << arc.center.x << " " << arc.center.y << "\n";
                std::cout << "   dir " << arc.dir << "\n";
                std::cout << "     n " << arc.points.size() << "\n";
                arc.points.clear();
                state = State::indeterminate;

            }
            return flush(all);
            break;
        }
    }
}

rs274_arcfit::rs274_arcfit()
 : rs274_base() {
     reset();
}

