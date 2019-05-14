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
 * rs274_arcfit.h
 *
 *  Created on: 2016-04-11
 *      Author: nicholas
 */

#ifndef RS274_ARCFIT_H_
#define RS274_ARCFIT_H_
#include "base/rs274_base.h"
#include "geometry_3.h"
#include <vector>
#include <boost/optional.hpp>

class rs274_arcfit : public rs274_base
{
private:
    struct block_point {
        block_t block;
        geometry_3::point_3 p0;
        geometry_3::point_3 p;
    };
    boost::optional<block_point> point_;
    geometry_3::point_3 to_point_3(const cxxcam::Position& pos);

    // state machine
    enum class State {
        indeterminate,
        collecting_points
    } state_;
    struct Arc {
        std::vector<block_point> points;
        geometry_3::vector_3 plane;
        double r;
        geometry_3::point_3 center;
        int dir;
        double arc_theta;

        enum {
            radiusDelta,
            chordHeight
        } minimise = radiusDelta;
    } arc_;
    double chord_height_tolerance;
    double point_deviation;
    double planar_tolerance;
    double theta_minimum;

    void reset();
    void push(const block_point& point);
    void flush(bool all = false);

    virtual void _rapid(const Position& pos);
    virtual void _arc(const Position& end, const Position& center, const cxxcam::math::vector_3& plane, int rotation);
    virtual void _linear(const Position& pos);
    virtual void block_end(const block_t& block);
    virtual void program_end();

public:
	rs274_arcfit(boost::program_options::variables_map& vm, double chord_height_tolerance, double point_deviation, double planar_tolerance, double theta_minimum);

	virtual ~rs274_arcfit() = default;
};

#endif /* RS274_ARCFIT_H_ */
