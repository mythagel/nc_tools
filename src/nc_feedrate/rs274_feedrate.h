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
 * rs274_feedrate.h
 *
 *  Created on: 2016-02-22
 *      Author: nicholas
 */

#ifndef RS274_FEEDRATE_H_
#define RS274_FEEDRATE_H_
#include "base/rs274_base.h"
#include "cxxcam/Position.h"
#include "cxxcam/Bbox.h"
#include <string>
#include <map>
#include "base/machine_config.h"
#include "clipper.hpp"

namespace cxxcam {
namespace path {
    struct step;
}
}

class rs274_feedrate : public rs274_base
{
private:
    cxxcam::Bbox _stock;
    // slice represents material from z - z+_slice_z
    std::map<double, ClipperLib::Paths> _slices;
    double _slice_z;

    struct {
        machine_config::mill_tool mill;
        machine_config::lathe_tool lathe;
    } _tool;

    double chip_load(const cxxcam::path::step& s0, const cxxcam::path::step& s1, double spindle_step);

    virtual void _rapid(const Position& pos);
    virtual void _arc(const Position& end, const Position& center, const cxxcam::math::vector_3& plane, int rotation);
    virtual void _linear(const Position& pos);
	virtual void tool_change(int slot);

public:
	rs274_feedrate(boost::program_options::variables_map& vm, const cxxcam::Bbox& stock, double slice_z);

	virtual ~rs274_feedrate() = default;
};

#endif /* RS274_MODEL_H_ */
