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
 * fold_adjacent.h
 *
 *  Created on: 2013-07-19
 *      Author: nicholas
 */

#ifndef FOLD_ADJACENT_H_
#define FOLD_ADJACENT_H_
#include <utility>

template<class InputIt, class OutputIt, class BinaryOperation>
OutputIt fold_adjacent(InputIt first, InputIt last, OutputIt d_first, BinaryOperation op)
{
	if (first == last)
		return d_first;

	auto acc = *first;
	while (++first != last)
	{
		auto val = *first;
		*++d_first = op(acc, val);
		acc = std::move(val);
	}
	return ++d_first;
}

#endif /* FOLD_ADJACENT_H_ */
