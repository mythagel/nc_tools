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
 * rs274_annotate.h
 *
 *  Created on: 2019-05-15
 *	  Author: nicholas
 */

#ifndef RS274_ANNOTATE_H_
#define RS274_ANNOTATE_H_
#include "base/rs274_base.h"
#include <string>
#include <vector>

class rs274_annotate : public rs274_base
{
private:
	virtual void _rapid(const Position& pos);
	virtual void _arc(const Position& end, const Position& center, const cxxcam::math::vector_3& plane, int rotation);
	virtual void _linear(const Position& pos);
	virtual void block_end(const block_t& block);

	void process_point(const cxxcam::math::point_3& p, bool rapid);

	std::vector<std::string> annotations_;

    // curvature
    std::vector<cxxcam::math::point_3> points;

public:
	rs274_annotate(boost::program_options::variables_map& vm);
	virtual ~rs274_annotate() = default;
};

#endif /* RS274_SHORTLINES_H_ */
