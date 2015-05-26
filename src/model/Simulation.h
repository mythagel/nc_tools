/* cxxcam - C++ CAD/CAM driver library.
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
 * Simulation.h
 *
 *  Created on: 2013-07-11
 *      Author: nicholas
 */

#ifndef SIMULATION_H_
#define SIMULATION_H_
#include <vector>
#include "Path.h"
#include "Tool.h"
#include "Stock.h"
#include "Units.h"
#include "Limits.h"
#include "Bbox.h"

namespace cxxcam
{
namespace simulation
{

/*
 * Sweep the tool along the path given, applying any needed transformations.
 */
geom::polyhedron_t sweep_tool(geom::polyhedron_t tool, const path::step& s0, const path::step& s1);

// TODO function to iterate path and validate feedrates
// TODO function to iterate path and calculate time

Bbox bounding_box(const std::vector<path::step>& steps);
geom::polyhedron_t remove_material(const geom::polyhedron_t& tool, const geom::polyhedron_t& stock, const std::vector<path::step>& steps);

/*
 * TODO simulation::run aggregates the individual functions above into a single interface.
 * TODO It is too easy with the current interface to forget to set a parameter.
 * Add a constructor to ensure fields are set.
 */
struct simulation_t
{
	path::path_t steps;
	
	Stock stock;
	Tool tool;
};
struct result_t
{
	Stock stock;
	
	Bbox bounding_box;
};
result_t run(const simulation_t& simulation);

}
}

#endif /* SIMULATION_H_ */
