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
 * Stock.h
 *
 *  Created on: 26/04/2012
 *      Author: nicholas
 */

#ifndef STOCK_H_
#define STOCK_H_
#include <memory>
#include "Material.h"
#include "geom/polyhedron.h"
#include <iosfwd>

namespace cxxcam
{

/*
 * Stores a description and model of the stock from which material will be removed.
 * Also should reference properties on the material the stock is made out of.
 */
struct Stock
{
	std::shared_ptr<material::Material> Material;
	geom::polyhedron_t Model;

	Stock() = default;
	Stock(const geom::polyhedron_t& model);
};

}

#endif /* STOCK_H_ */
