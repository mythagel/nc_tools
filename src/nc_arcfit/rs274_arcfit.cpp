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
#include <algorithm>

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
    point->l.a = to_point_3(convert(program_pos));
    point->l.b = to_point_3(convert(pos));
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

template <typename T>
bool equal(T a, T b, T tolerance) {
    return std::abs(b-a) < tolerance;
}
void rs274_arcfit::push(const block_point& point) {
    std::cerr << "\n\npush: " << point.l.b << "\n";
    switch (state)
    {
        case State::indeterminate:
        {
            arc.points.push_back(point);

            if (arc.points.size() < 2)
                return;
            auto p0 = arc.points[0].l.a;
            auto p1 = arc.points[0].l.b;
            auto p2 = arc.points[1].l.b;
std::cerr << "p0 " << p0 << " p1 " << p1 << " p2 " << p2 << "\n";

            if (collinear(p0, p1, p2, 1e-6))
                return flush();

            // TODO handle XZ and YZ planes
            // remembering to map to xy when calculating circle center!
            arc.plane = geometry_3::plane(p0, p1, p2);
std::cerr << "pl " << arc.plane.x << " " << arc.plane.y << " " << arc.plane.z << "\n";
            if (! equal(std::abs(arc.plane.z), 1.0, arc.planar_tolerance))
                return flush();

            auto center = circle_center(p0, p1, p2);
std::cerr << "center " << center->x << " " << center->y << "\n";
            if (!center)
                return flush();
            arc.center = *center;

std::cerr << "p0 " << p0.x << " " << p0.y << "\n";
std::cerr << "p1 " << arc.center.x << " " << arc.center.y << "\n";
            arc.r = std::abs(distance(p0, arc.center));
            std::cerr << "radius: " << arc.r << "\n";

            auto h0 = chord_height(p0, p1, arc.r);
            auto h1 = chord_height(p1, p2, arc.r);
            std::cerr << "h0: " << h0 << " h1 " << h1 << "\n";
            if (h0 > arc.chord_height_tolerance || h1 > arc.chord_height_tolerance)
                return flush();

            // need to identify direction, and initial theta
            auto t0 = theta(p0, arc.center);
            auto t1 = theta(p1, arc.center);
            std::cerr << "t0: " << t0 << " t1 " << t1 << "\n";

            arc.dir = boost::math::sign(t1 - t0);
            std::cerr << "dir: " << arc.dir << "\n";

            state = State::collecting_points;
            break;
        }
        case State::collecting_points:
        {
            auto radius_delta_sum = [](const std::vector<block_point>& points, geometry_3::point_3 center) {
                double sum = 0;
                double radius = 0;

                if (!points.empty()) {
                    auto& point = points[0];
                    radius = std::abs(distance(point.l.a, center));
                }

                for (auto& point : points) {
                    auto r = std::abs(distance(point.l.b, center));
                    sum += std::abs(r - radius);
                }
                return sum;
            };
            auto chord_height_sum = [](const std::vector<block_point>& points, double r) {
                double chs = 0;
                for (auto& bp : points)
                    chs += chord_height(bp.l.a, bp.l.b, r);
                return chs;
            };

            {
                auto p0 = arc.points[0].l.a;
                // ...
                auto pmid = arc.points[arc.points.size()/2].l.b;
                // ...
                auto pn_1 = arc.points[arc.points.size()-1].l.b;
                auto pn = point.l.b;

                auto center = circle_center(p0, pmid, pn_1);
                auto dr0 = radius_delta_sum(arc.points, arc.center);
                auto dr1 = radius_delta_sum(arc.points, *center);

                auto r0 = std::abs(distance(pn, arc.center));
                auto r1 = std::abs(distance(pn, *center));

                auto ch0 = chord_height_sum(arc.points, r0);
                auto ch1 = chord_height_sum(arc.points, r1);

                std::cerr << "r0: " << r0 << " r1 " << r1 << "\n";
                std::cerr << "dr0: " << dr0 << " dr1 " << dr1 << "\n";
                std::cerr << "ch0: " << ch0 << " ch1 " << ch1 << "\n";
                std::cerr << "center2 " << center->x << " " << center->y << "\n";

                if (arc.minimise == Arc::radiusDelta) {
                    if (dr1 < dr0) {
                        arc.center = *center;
                        arc.r = r1;
                    }
                } else if (arc.minimise == Arc::chordHeight) {
                    if (ch1 < ch0) {
                        arc.center = *center;
                        arc.r = r1;
                    }
                }
            }

            auto pn_1 = arc.points.back().l.b;
            auto pn = point.l.b;

            auto r = std::abs(distance(pn, arc.center));
            std::cerr << "r: " << r << " arc.r " << arc.r << "\n";
            if (std::abs(arc.r - r) > arc.point_deviation)
                flush();

            // TODO find exact point on arc relative to arc point tolerance
            
            auto ch = chord_height(pn_1, pn, arc.r);
            std::cerr << "ch: " << ch << "\n";
            if (ch > arc.chord_height_tolerance)
                return flush();

            auto t0 = theta(pn_1, arc.center);
            auto t1 = theta(pn, arc.center);
            std::cerr << "t0: " << t0 << " t1 " << t1 << "\n";

            auto dir = boost::math::sign(t1 - t0);
            std::cerr << "dir: " << dir << "\n";
            if (dir != arc.dir)
                return flush();

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
            auto flush_arc = [&]() {
                // TODO flush arc if theta delta is > threshold
                return arc.points.size() > 3;
            };
            if (flush_arc()) {
                block_t block;

                if (arc.dir == 1)
                    block.g_modes[1] = 20; // G2
                else
                    block.g_modes[1] = 30; // G3


                auto p0 = arc.points[0].l.a;
                auto p1 = arc.points[arc.points.size()-1].l.b;

                block.x = p1.x;
                block.y = p1.y;
                block.i = arc.center.x - p0.x;
                block.j = arc.center.y - p0.y;
                block.f = _feed_rate;

                std::cerr << "arc p0 " << p0 << "\n";
                std::cerr << "    p1 " << p1 << "\n";
                std::cerr << "    pc " << arc.center << "\n";
                std::cerr << "   dir " << arc.dir << "\n";
                std::cerr << "     n " << arc.points.size() << "\n";
                std::cout << str(block) << "\n";
                arc.points.clear();
            }
            state = State::indeterminate;
            return flush(all);
            break;
        }
    }
}

rs274_arcfit::rs274_arcfit()
 : rs274_base() {
     reset();
}

