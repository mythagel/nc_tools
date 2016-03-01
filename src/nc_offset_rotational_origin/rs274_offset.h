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
 * rs274_offset.h
 *
 *  Created on: 2015-11-12
 *      Author: nicholas
 */

#ifndef RS274_OFFSET_H_
#define RS274_OFFSET_H_
#include "base/rs274_base.h"
#include "Bbox.h"

/* allow ABC to be redefined around arbitary axes?
 * Or enforce AX BY CZ correlation
 * AXBYCZ requires only 3 points for axis center
 *
 * otherwise 3 vectors are required to define axis + center.
 *
 * initially enforce AXBYCZ
 * */

struct rotational_origin
{
    bool tool;
    double x;
    double y;
    double z;

    rotational_origin()
     : tool(true)
    {}
    rotational_origin(double x, double y, double z)
     : tool(false), x(x), y(y), z(z)
    {}
};

class rs274_offset : public rs274_base
{
private:
    rotational_origin from_;
    rotational_origin to_;

    virtual void _rapid(const Position& pos);
    virtual void _arc(const Position& end, const Position& center, const cxxcam::math::vector_3& plane, int rotation);
    virtual void _linear(const Position& pos);
    virtual void block_end();

public:
	rs274_offset(const rotational_origin& from, const rotational_origin& to);

	virtual ~rs274_offset() = default;
};

#endif /* RS274_OFFSET_H_ */
