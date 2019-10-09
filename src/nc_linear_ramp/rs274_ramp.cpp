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
 * rs274_ramp.cpp
 *
 *  Created on: 2019-10-04
 *      Author: nicholas
 */

#include "rs274_ramp.h"
#include <iostream>
#include "cxxcam/Path.h"
#include "base/machine_config.h"
#include "../r6.h"
#include "../throw_if.h"
#include "../fold_adjacent.h"
#include <numeric>

void rs274_ramp::_rapid(const Position&) {
}

void rs274_ramp::_arc(const Position&, const Position&, const cxxcam::math::vector_3&, int) {
    throw_if(m_ramping, "Arc not handled while ramping");
}

void rs274_ramp::_linear(const Position& pos) {
    using namespace cxxcam::path;
    using cxxcam::units::length_mm;
    using cxxcam::units::length_inch;

    // detect plunge movement
    if (std::abs(pos.x - program_pos.x) < 1e9 &&
            std::abs(pos.y - program_pos.y) < 1e9 &&
            (program_pos.z - pos.z) > 0)
    {
        throw_if(m_ramping, "Plunge while ramping not supported");

        m_ramping = true;
        m_start = convert(program_pos);
        m_ramp = convert(pos);
        m_rampPath.clear();
    }

    if (m_ramping) {
        throw_if (std::abs((convert(pos).Z - m_ramp.Z).value()) > 1e9, "Ramping on 3D path not supported.");

        m_rampPath.push_back(convert(pos));

        auto deg2rad = [](double d) { return (d / 180.0) * PI; };

        if (length_mm(ramp_path_length()).value() >= m_length) {

            // Hit or exceeded target length, output ramping

            std::vector<cxxcam::path::step> ramp_steps;
            // TODO trim to exact target length
            {
                cxxcam::Position prev = m_rampPath.front();
                for (auto& pos : m_rampPath) {
                    auto steps = expand_linear(prev, pos, {}, 5).path;
                    ramp_steps.insert(end(ramp_steps), begin(steps), end(steps));
                    prev = pos;
                }
            }

            auto Z = m_start.Z;
            int step_index = 0;
            int direction = 1;

            cxxcam::path::step prev = ramp_steps.front();
            cxxcam::units::length dist;
            cxxcam::units::length h(0.5 * cxxcam::units::millimeter);   // TODO calculate

            while (Z > m_ramp.Z) {
                auto step = ramp_steps[step_index];
                auto step_delta = cxxcam::math::distance(prev.position, step.position);

                auto z = (std::sin(deg2rad(m_angle)) * step_delta);
                if (z.value() < 0)
                    z = -z;
                Z -= z;
                if (Z <= m_ramp.Z)
                    Z = m_ramp.Z;

                step.position.z = Z;

                output_point(step.position);

                dist += step_delta;
                prev = ramp_steps[step_index];

                step_index += direction;
                if (static_cast<unsigned>(step_index) == ramp_steps.size() || step_index == -1) {
                    direction = -direction;
                    step_index += direction*2;
                    Z += h;
                }
            }

            while (true) {
                auto step = ramp_steps[step_index];
                output_point(step.position);

                if (direction == 1)
                    step_index -= direction;    // Reversed
                else
                    step_index += direction;
                if (static_cast<unsigned>(step_index) == ramp_steps.size() || step_index == -1)
                    break;
            }

            {
                cxxcam::Position prev = m_rampPath.front();
                for (auto& pos : m_rampPath) {
                    auto steps = expand_linear(prev, pos, {}, -1).path;
                    for (auto& step : steps) {
                        auto& p = step.position;
                        output_point(p);
                    }
                    prev = pos;
                }
            }

            m_ramping = false;
        }
    }
}

cxxcam::units::length rs274_ramp::ramp_path_length() {
    using namespace cxxcam;
    std::vector<cxxcam::units::length> lengths;
    fold_adjacent(begin(m_rampPath), end(m_rampPath), std::back_inserter(lengths), cxxcam::path::length_linear);
    return std::accumulate(begin(lengths), end(lengths), cxxcam::units::length{});
}

void rs274_ramp::output_point(const cxxcam::math::point_3& p) const {
    using cxxcam::units::length_mm;
    using cxxcam::units::length_inch;

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

    std::cout << " F" << _feed_rate;
    std::cout << "\n";
}

void rs274_ramp::block_end(const block_t& block) {
    if (m_ramping == false) {
        std::cout << str(block) << "\n";
    }
}
rs274_ramp::rs274_ramp(boost::program_options::variables_map& vm)
 : rs274_base(vm) {
    m_angle = vm["angle"].as<double>();
    m_length = vm["length"].as<double>();
    m_ramping = false;

    throw_if(m_length == 0, "Zero ramp length");
}

