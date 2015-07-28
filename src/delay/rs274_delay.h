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
 * rs274_delay.h
 *
 *  Created on: 2015-07-24
 *      Author: nicholas
 */

#ifndef RS274_DELAY_H_
#define RS274_DELAY_H_
#include "nc_base/rs274_base.h"

class rs274_delay : public rs274_base
{
private:
    virtual void _rapid(const Position& pos);
    virtual void _arc(const Position& end, const Position& center, const cxxcam::math::vector_3& plane, int rotation);
    virtual void _linear(const Position& pos);

public:
	rs274_delay();
	virtual ~rs274_delay() = default;
};

#endif /* RS274_DELAY_H_ */
