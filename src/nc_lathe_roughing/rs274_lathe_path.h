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
 * rs274_path.h
 *
 *  Created on: 2016-03-24
 *      Author: nicholas
 */

#ifndef RS274_PATH_H_
#define RS274_PATH_H_
#include "base/rs274_base.h"
#include "geometry.h"
#include "clipper.hpp"

class rs274_path : public rs274_base
{
private:

    virtual void _rapid(const Position&);
    virtual void _arc(const Position& end, const Position& center, const cxxcam::math::vector_3& plane, int rotation);
    virtual void _linear(const Position& pos);

    struct {
        double x;
        double z;
    } start_point_;

    ClipperLib::Path path_;
public:
	rs274_path();

    double start_x() const { return start_point_.x; }
    double start_z() const { return start_point_.z; }
    ClipperLib::Path path() const;

    ClipperLib::IntPoint scale_point(const cxxcam::math::point_3& p) const;
    double scale() const {
        return 10e12;
    }

	virtual ~rs274_path() = default;
};

#endif /* RS274_PATH_H_ */
