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
 * rs274_ramp.h
 *
 *  Created on: 2019-10-04
 *      Author: nicholas
 */

#ifndef RS274_RAMP_H_
#define RS274_RAMP_H_
#include "base/rs274_base.h"

class rs274_ramp : public rs274_base
{
private:

    virtual void _rapid(const Position& pos);
    virtual void _arc(const Position& end, const Position& center, const cxxcam::math::vector_3& plane, int rotation);
    virtual void _linear(const Position& pos);

    cxxcam::units::length ramp_path_length();

    virtual void block_end(const block_t& block);

    void output_point(const cxxcam::math::point_3& p) const;
private:
    double m_angle;
    double m_length;

    bool m_ramping;
    std::vector<cxxcam::Position> m_rampPath;
    cxxcam::Position m_start;
    cxxcam::Position m_ramp;

public:
	rs274_ramp(boost::program_options::variables_map& vm);

	virtual ~rs274_ramp() = default;
};

#endif /* RS274_RAMP_H_ */
