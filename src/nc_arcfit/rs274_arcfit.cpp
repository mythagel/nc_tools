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
#include "cxxcam/Units.h"
#include <boost/math/special_functions/sign.hpp>
#include <algorithm>
#include "base/machine_config.h"

geometry_3::point_3 rs274_arcfit::to_point_3(const cxxcam::Position& pos) {
    using cxxcam::units::length_mm;
    using cxxcam::units::length_inch;
    using namespace machine_config;

    switch (machine_units(config, machine_id)) {
        case machine_config::units::metric:
            return {length_mm(pos.X).value(), length_mm(pos.Y).value(), length_mm(pos.Z).value()};
        case machine_config::units::imperial:
            return {length_inch(pos.X).value(), length_inch(pos.Y).value(), length_inch(pos.Z).value()};
        default:
            throw std::logic_error("Unhandled units");
    }
}

void rs274_arcfit::_rapid(const Position&) {
    point_ = {};
}

void rs274_arcfit::_arc(const Position&, const Position&, const cxxcam::math::vector_3&, int) {
    point_ = {};
}

void rs274_arcfit::_linear(const Position& pos) {
    // record motion point here ...
    point_ = block_point();
    point_->p0 = to_point_3(convert(program_pos));
    point_->p = to_point_3(convert(pos));
}

void rs274_arcfit::block_end(const block_t& block) {
    // ... but process it here iff the block describes simple linear motion
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

    
    if (is_linear(block) && point_) {
        point_->block = block;
        push(*point_);
    } else {
        flush(true);
        std::cout << str(block) << "\n";
    }
    point_ = {};
}
void rs274_arcfit::program_end() {
    flush(true);
}

void rs274_arcfit::reset() {
    state_ = State::indeterminate;
}

template <typename T>
bool equal(T a, T b, T tolerance) {
    return std::abs(b-a) < tolerance;
}
void rs274_arcfit::push(const block_point& point) {
    static const double PI = 3.14159265358979323846;

    auto radius_delta_sum = [](const std::vector<block_point>& points, geometry_3::point_3 center) {
        double sum = 0;
        double radius = 0;

        if (!points.empty()) {
            auto& point = points[0];
            radius = std::abs(distance(point.p, center));
        }

        for (auto& point : points) {
            auto r = std::abs(distance(point.p, center));
            sum += std::abs(r - radius);
        }
        return sum;
    };

    auto chord_height_sum = [](const std::vector<block_point>& points, double r) {
        double chs = 0;
        auto prev = points[0];
        for (auto& bp : points) {
            chs += chord_height(prev.p, bp.p, r);
            prev = bp;
        }
        return chs;
    };

    auto arc_direction = [&](const geometry_3::point_3& p0, const geometry_3::point_3& p1) {
        auto a = p0 - arc_.center;
        auto b = p1 - arc_.center;
        auto dir = boost::math::sign(a.x*b.y - a.y*b.x);
        return -dir;
    };

    auto delta_theta = [&] (const geometry_3::point_3& p0, const geometry_3::point_3& p1, int dir) {
        auto t0 = theta(p0, arc_.center);
        auto t1 = theta(p1, arc_.center);
        auto dt = t1 - t0;
        switch(dir)
        {
            case 1:
            {
                if(dt > 0)
                    dt -= PI*2;
                else if(dt == 0)
                    dt = -PI*2;
                break;
            }
            case -1:
            {
                if(dt < 0)
                    dt += PI*2;
                else if(dt == 0)
                    dt = PI*2;
                break;
            }
        }
        return dt;
    };

    switch (state_)
    {
        case State::indeterminate:
        {
            arc_.points.push_back(point);

            if (arc_.points.size() < 3)
                return;
            auto p0 = arc_.points[0].p0;
            auto p1 = arc_.points[0].p;
            auto p2 = arc_.points[1].p;

            // 0. Verify points are not collinear
            if (collinear(p0, p1, p2, 1e-6))
                return flush();

            // 1. Determine arc plane
            // TODO handle XZ and YZ planes
            // remembering to map to xy when calculating circle center!
            arc_.plane = geometry_3::plane(p0, p1, p2);
            if (! equal(std::abs(arc_.plane.z), 1.0, planar_tolerance))
                return flush();

            // 2. Determine center point
            auto center = circle_center(p0, p1, p2);
            if (!center)
                return flush();
            arc_.center = *center;

            // 3. Determine radius
            arc_.r = std::abs(distance(p0, arc_.center));

            // 4. Validate chord height
            auto h0 = chord_height(p0, p1, arc_.r);
            auto h1 = chord_height(p1, p2, arc_.r);
            if (h0 > chord_height_tolerance || h1 > chord_height_tolerance)
                return flush();

            // 5. Determine arc direction
            arc_.dir = arc_direction(p0, p1);

            // 4. Update arc theta
            arc_.arc_theta = delta_theta(p0, p1, arc_.dir);

            state_ = State::collecting_points;
            break;
        }
        case State::collecting_points:
        {
            auto pn_2 = arc_.points[arc_.points.size() - 2].p;
            auto pn_1 = arc_.points[arc_.points.size() - 1].p;
            auto pn = point.p;

            // 0. Verify point is not collinear
            if (collinear(pn_2, pn_1, pn, 1e-6))
                return flush();

            // 1. Verify arc plane
            // TODO handle XZ and YZ planes
            // remembering to map to xy when calculating circle center!
            auto plane = geometry_3::plane(pn_2, pn_1, pn);
            if (! equal(std::abs(plane.z), std::abs(arc_.plane.z), planar_tolerance))
                return flush();

            // 2. Center point is known

            // 3. Verify point radius
            auto r = std::abs(distance(pn, arc_.center));
            if (std::abs(arc_.r - r) > point_deviation)
                flush();

            // TODO find exact point on arc relative to arc point tolerance
            
            // 4. Validate chord height
            auto ch = chord_height(pn_1, pn, arc_.r);
            if (ch > chord_height_tolerance)
                return flush();

            // 3. Determine arc direction
            auto dir = arc_direction(pn_1, pn);
            if (dir != arc_.dir)
                return flush();

            // 4. Update arc theta
            auto dt = delta_theta(pn_1, pn, arc_.dir);

            //if (arc.arc_theta + dt > 2*PI)
            //    return flush();

            {
                auto p0 = arc_.points[0].p0;
                // ...
                auto pmid = arc_.points[arc_.points.size()/2].p;
                // ...
                auto pn_1 = arc_.points[arc_.points.size() - 1].p;
                auto pn = point.p;

                auto center = circle_center(p0, pmid, pn_1);
                if (! center)
                    return flush();

                auto r0 = std::abs(distance(pn, arc_.center));
                auto r1 = std::abs(distance(pn, *center));

                if (arc_.minimise == Arc::radiusDelta) {
                    auto dr0 = radius_delta_sum(arc_.points, arc_.center);
                    auto dr1 = radius_delta_sum(arc_.points, *center);

                    if (dr1 < dr0) {
                        arc_.center = *center;
                        arc_.r = r1;
                    }
                } else if (arc_.minimise == Arc::chordHeight) {
                    auto ch0 = chord_height_sum(arc_.points, r0);
                    auto ch1 = chord_height_sum(arc_.points, r1);

                    if (ch1 < ch0) {
                        arc_.center = *center;
                        arc_.r = r1;
                    }
                }
            }

            arc_.arc_theta += dt;
            arc_.points.push_back(point);
            break;
        }
    }
}
void rs274_arcfit::flush(bool all) {
    switch (state_)
    {
        case State::indeterminate:
        {
            while (!arc_.points.empty()) {
                auto& block = arc_.points[0].block;
                std::cout << str(block) << "\n";
                arc_.points.erase(begin(arc_.points));

                if (!all) break;
            }
            break;
        }
        case State::collecting_points:
        {
            auto flush_arc = [&]() {
                return arc_.arc_theta > theta_minimum;
            };
            if (flush_arc()) {
                block_t block;

                if (arc_.dir == 1)
                    block.g_modes[1] = 20; // G2
                else
                    block.g_modes[1] = 30; // G3


                auto p0 = arc_.points[0].p0;
                auto pn = arc_.points[arc_.points.size()-1].p;

                // Calculate radius to higher precision
                arc_.r = std::abs(distance(arc_.points[0].p0, arc_.center));
                for (auto& block : arc_.points)
                    arc_.r += std::abs(distance(block.p, arc_.center));
                arc_.r /= (arc_.points.size() + 1);

                auto map_units = [&](double value) {
                    using namespace cxxcam::units;
                    using namespace machine_config;
                    length x;
                    switch (machine_units(config, machine_id)) {
                        case machine_config::units::metric:
                            x = length{ value * millimeters };
                            break;
                        case machine_config::units::imperial:
                            x = length{ value * inches };
                            break;
                        default:
                            throw std::logic_error("Unhandled default units");
                    }
                    switch (_length_unit_type) {
                        case Units::Metric:
                            return length_mm(x).value();
                        case Units::Imperial:
                            return length_inch(x).value();
                        default:
                            throw std::logic_error("Unhandled nc units");
                    }
                };
                // TODO use current arc incremental / absolute mode
                block.x = map_units(pn.x);
                block.y = map_units(pn.y);
                block.i = map_units(arc_.center.x - p0.x);
                block.j = map_units(arc_.center.y - p0.y);
                block.f = _feed_rate;

                std::cout << str(block) << "\n";
                arc_.points.clear();
            }
            state_ = State::indeterminate;
            return flush(all);
            break;
        }
    }
}

rs274_arcfit::rs274_arcfit(boost::program_options::variables_map& vm, double chord_height_tolerance, double point_deviation, double planar_tolerance, double theta_minimum)
 : rs274_base(vm), chord_height_tolerance(chord_height_tolerance), point_deviation(point_deviation), planar_tolerance(planar_tolerance), theta_minimum(theta_minimum) {
     reset();
}

