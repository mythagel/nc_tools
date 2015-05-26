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
 * Tool.cpp
 *
 *  Created on: 26/04/2012
 *      Author: nicholas
 */

#include "Tool.h"
#include "geom/primitives.h"
#include "Error.h"

namespace cxxcam
{

namespace
{

geom::polyhedron_t make_mill_tool(const Tool::Mill& em)
{
	using namespace geom;

	// TODO determine slices based on height under arc + accuracy.
	auto shank = make_cone( {0, 0, em.length}, {0, 0, em.cutting_length}, em.shank_diameter, em.shank_diameter, 8);
	
	switch(em.type)
	{
		case Tool::Mill::Type::End:
		{
			auto flutes = make_cone( {0, 0, em.cutting_length}, {0, 0, 0}, em.mill_diameter, em.mill_diameter, 8);
			return shank + flutes;
		}
		default:
			throw error("Unimplemented mill type.");
	}

// Shank tapers to cutting diameter.
//	auto shank = make_cone(0, 0, em.cutting_length+em.shank_length, 0, 0, em.cutting_length, em.shank_diameter, em.cutting_diameter, 64);

	throw error("Unable to create tool geometry model");
}

}

Tool::Tool()
 : m_Name("Invalid")
{
}

Tool::Tool(const std::string& name, const Mill& mill)
 : m_Name(name), m_Type(Type::Mill), m_Mill(mill), m_Model(make_mill_tool(m_Mill))
{
}
Tool::Tool(const std::string& name, const Lathe& lathe)
 : m_Name(name), m_Type(Type::Lathe), m_Lathe(lathe)
{
}

std::string Tool::Name() const
{
	return m_Name;
}

Tool::Type Tool::ToolType() const
{
	return m_Type;
}

auto Tool::GetMill() const -> Mill
{
	if(m_Type != Type::Mill)
		throw error("GetMill: Tool is not Mill type.");
	
	return m_Mill;
}

auto Tool::GetLathe() const -> Lathe
{
	if(m_Type != Type::Lathe)
		throw error("GetLathe: Tool is not Lathe type.");
	
	return m_Lathe;
}

geom::polyhedron_t Tool::Model() const
{
	return m_Model;
}

}

