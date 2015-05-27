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
 * Simulation.cpp
 *
 *  Created on: 2013-07-11
 *      Author: nicholas
 */

#include "Simulation.h"
#include "geom/translate.h"
#include "geom/ops.h"
#include "geom/query.h"
#include "fold_adjacent.h"
#include <numeric>
#include <future>

namespace cxxcam
{
namespace simulation
{

geom::polyhedron_t sweep_tool(geom::polyhedron_t tool, const path::step& s0, const path::step& s1)
{
	using units::length_mm;

	static const math::quaternion_t identity{1,0,0,0};

	const auto& o0 = s0.orientation;
	const auto& p0 = s0.position;
	const auto& p1 = s1.position;

	if(o0 != identity)
		tool = geom::rotate(tool, o0.R_component_1(), o0.R_component_2(), o0.R_component_3(), o0.R_component_4());

	if(distance(p0, p1) > units::length{0.000001 * units::millimeters})
	{
		geom::polyline_t path{ { {length_mm(p0.x).value(), length_mm(p0.y).value(), length_mm(p0.z).value()}, 
								{length_mm(p1.x).value(), length_mm(p1.y).value(), length_mm(p1.z).value()} } };
		return geom::glide(tool, path);
	}
	
	return translate(tool, length_mm(p0.x).value(), length_mm(p0.y).value(), length_mm(p0.z).value());
}

Bbox bounding_box(const std::vector<path::step>& steps)
{
	if(steps.empty())
		return {};
	
	auto s0 = steps.front();
	return std::accumulate(++begin(steps), end(steps), Bbox{s0.position, s0.position}, [](Bbox& b, const path::step& s0) { return b + s0.position; } );
}

geom::polyhedron_t remove_material(const geom::polyhedron_t& tool, const geom::polyhedron_t& stock, const std::vector<path::step>& steps)
{
	auto fold_path = [&tool](std::vector<path::step>::const_iterator begin, std::vector<path::step>::const_iterator end) -> geom::polyhedron_t
	{
		std::vector<geom::polyhedron_t> tool_motion;
		
		fold_adjacent(begin, end, std::back_inserter(tool_motion), 
		[&tool](const path::step& s0, const path::step& s1) -> geom::polyhedron_t
		{
			return sweep_tool(tool, s0, s1);
		});
		
		return geom::merge(tool_motion);
	};

	auto tool_path = fold_path(begin(steps), end(steps));
	return stock - tool_path;
}

result_t run(const simulation_t& simulation)
{
	result_t result;
	
	result.stock = remove_material(simulation.tool.Model(), simulation.stock.Model, simulation.steps.path);
	result.bounding_box = bounding_box(simulation.steps.path);
	
	return result;
}

}
}

