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
#include "../fold_adjacent.h"
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
template <typename T>
bool equal(T a, T b, T tolerance) {
    return std::abs(b-a) < tolerance;
}
void rs274_arcfit::push(const block_point& point) {
    switch (state)
    {
        case State::indeterminate:
        {
            arc.points.push_back(point);

            if (arc.points.size() < 3)
                return;
            std::cerr << "\n";
            std::cerr << "\n";
            auto p0 = arc.points[0].p;
            auto p1 = arc.points[1].p;
            auto p2 = arc.points[2].p;
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
            auto t0 = std::atan((p0.y - arc.center.y) / (p0.x - arc.center.x));
            auto t1 = std::atan((p1.y - arc.center.y) / (p1.x - arc.center.x));
            std::cerr << "t0: " << t0 << " t1 " << t1 << "\n";

            arc.dir = boost::math::sign(t0 - t1);
            std::cerr << "dir: " << arc.dir << "\n";

            state = State::collecting_points;
            break;
        }
        case State::collecting_points:
        {
            std::cerr << "\n";
            auto radius_delta_sum = [](const std::vector<block_point>& points, geometry_3::point_3 center) {
                double sum = 0;
                double radius = 0;
                for (auto& point : points) {
                    auto r = std::abs(distance(point.p, center));
                    if (radius == 0) radius = r;
                    sum += std::abs(r - radius);
                }
                return sum;
            };
            auto chord_height_sum = [](const std::vector<block_point>& points, double r) {
                std::vector<double> chord_heights;
                fold_adjacent(std::begin(points), std::end(points), std::back_inserter(chord_heights), 
                        [&](const block_point& bp0, const block_point& bp1) {
                            return chord_height(bp0.p, bp1.p, r);
                        });
                return std::accumulate(begin(chord_heights), end(chord_heights), 0.0);
            };
            auto pn_1 = arc.points.back().p;
            auto pn = point.p;

            {
                auto center = circle_center(arc.points[0].p, arc.points[arc.points.size()/2].p, arc.points[arc.points.size()-1].p);
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

            auto r = std::abs(distance(pn, arc.center));
            std::cerr << "r: " << r << " arc.r " << arc.r << "\n";
            if (std::abs(arc.r - r) > arc.point_deviation)
                flush();

            // TODO find exact point on arc relative to arc point tolerance
            
            auto ch = chord_height(pn_1, pn, arc.r);
            std::cerr << "ch: " << ch << "\n";
            if (ch > arc.chord_height_tolerance)
                return flush();

            auto t0 = std::atan((pn_1.y - arc.center.y) / (pn_1.x - arc.center.x));
            auto t1 = std::atan((pn.y - arc.center.y) / (pn.x - arc.center.x));
            std::cerr << "t0: " << t0 << " t1 " << t1 << "\n";

            auto dir = boost::math::sign(t0 - t1);
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
            if (arc.points.size() > 3) {
                block_t block;

                if (arc.dir == 1)
                    block.g_modes[1] = 20; // G2
                else
                    block.g_modes[1] = 30; // G3


                auto p0 = arc.points[0].p;
                auto p1 = arc.points[arc.points.size()-1].p;

                block.x = p1.x;
                block.y = p1.y;
                block.i = arc.center.x - p1.x;
                block.j = arc.center.y - p1.y;
                block.f = _feed_rate;

                std::cerr << "arc p0 " << p0.x << " " << p0.y << "\n";
                std::cerr << "    p1 " << p1.x << " " << p1.y << "\n";
                std::cerr << "    pc " << arc.center.x << " " << arc.center.y << "\n";
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

