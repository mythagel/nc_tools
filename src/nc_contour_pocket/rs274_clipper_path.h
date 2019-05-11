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
 * rs274_clipper_path.h
 *
 *  Created on: 2016-04-07
 *      Author: nicholas
 */

#ifndef RS274_CLIPPER_PATH_H_
#define RS274_CLIPPER_PATH_H_
#include "base/rs274_base.h"
#include "cxxcam/Math.h"
#include "clipper.hpp"
#include <functional>

class rs274_clipper_path : public rs274_base
{
private:

    virtual void _rapid(const Position&);
    virtual void _arc(const Position& end, const Position& center, const cxxcam::math::vector_3& plane, int rotation);
    virtual void _linear(const Position& pos);
    virtual void program_end();

    ClipperLib::Paths path_;
    std::function<void(ClipperLib::Paths, double z)> new_path_;
public:
	rs274_clipper_path(boost::program_options::variables_map& vm);

    ClipperLib::Paths path() const;
    void set_callback(std::function<void(ClipperLib::Paths, double z)> new_path);

    ClipperLib::IntPoint scale_point(const cxxcam::math::point_3& p) const;
    double scale() const {
        return 10e12;
    }

	virtual ~rs274_clipper_path() = default;
};

#endif /* RS274_CLIPPER_PATH_H_ */
