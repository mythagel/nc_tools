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
 * rs274_model.h
 *
 *  Created on: 2015-05-22
 *      Author: nicholas
 */

#ifndef RS274_MODEL_H_
#define RS274_MODEL_H_
#include "base/rs274_base.h"
#include "Position.h"
#include "geom/polyhedron.h"
#include <string>
#include <vector>

class rs274_model : public rs274_base
{
private:
    geom::polyhedron_t _model;
    geom::polyhedron_t _tool;
    std::vector<geom::polyhedron_t> _toolpath;
    unsigned _steps_per_revolution = 360;
    bool _lathe = false;

    virtual void _rapid(const Position& pos);
    virtual void _arc(const Position& end, const Position& center, const cxxcam::math::vector_3& plane, int rotation);
    virtual void _linear(const Position& pos);
	virtual void tool_change(int slot);
	virtual void dwell(double seconds);

public:
	rs274_model(const std::string& stock_filename);

    geom::polyhedron_t model();

	virtual ~rs274_model() = default;
};

#endif /* RS274_MODEL_H_ */
