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
 * machine_config.h
 *
 *  Created on: 2016-04-30
 *      Author: nicholas
 */

#ifndef MACHINE_CONFIG_H_
#define MACHINE_CONFIG_H_
#include <string>
#include "nc_config.h"

namespace machine_config {

std::string default_machine(nc_config& config);

enum class machine_type {
    mill,
    lathe
};
machine_type get_machine_type(nc_config& config, const std::string& machine);

struct mill_tool {
    std::string name;
    double length = 0.0;
    double diameter = 0.0;
    double flute_length = 0.0;
    double shank_diameter = 0.0;
};
struct lathe_tool {
    std::string name;
};

bool get_tool(nc_config& config, unsigned id, const std::string& machine, mill_tool& tool);
bool get_tool(nc_config& config, unsigned id, const std::string& machine, lathe_tool& tool);

}

#endif /* MACHINE_CONFIG_H_ */
