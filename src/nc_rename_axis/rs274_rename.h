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
 * rs274_rename.h
 *
 *  Created on: 2016-04-01
 *      Author: nicholas
 */

#ifndef RS274_RENAME_H_
#define RS274_RENAME_H_
#include "base/rs274_base.h"
#include <iostream>
#include <vector>

struct AxisModification
{
    enum Axis
    {
        axis_None,
        axis_X,
        axis_Y,
        axis_Z,
        axis_A,
        axis_B,
        axis_C,
        axis_I,
        axis_J,
        axis_K,
    };
    static Axis map(char c);

    Axis from = axis_None;
    Axis to = axis_None;
};
inline std::istream& operator>>(std::istream& is, AxisModification& a)
{
    char c;
    if(is >> c) a.from = AxisModification::map(c);
    if(is.peek() != EOF && is >> c) a.to = AxisModification::map(c);
    return is;
}

class rs274_rename : public rs274_base
{
private:
    std::vector<AxisModification> mods;
    void apply_mods(block_t& block) const;

    virtual void _rapid(const Position& pos);
    virtual void _arc(const Position& end, const Position& center, const cxxcam::math::vector_3& plane, int rotation);
    virtual void _linear(const Position& pos);

    virtual void block_end(const block_t& block);

public:
	rs274_rename(const std::vector<AxisModification>& mods);

	virtual ~rs274_rename() = default;
};

#endif /* RS274_RENAME_H_ */
