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
 * nc_config.h
 *
 *  Created on: 2016-02-09
 *      Author: nicholas
 */

#ifndef NC_CONFIG_H_
#define NC_CONFIG_H_
#include "lua/state.h"
#include <string>

struct nc_config
{
protected:
    lua::state L;
public:
    nc_config(const std::string& conf = {});
    nc_config(nc_config&) = delete;
    nc_config& operator=(nc_config&) = delete;

    inline lua::state& state() { return L; }
};

#endif /* NC_CONFIG_H_ */
