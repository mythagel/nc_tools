/* 
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
 * throw_if.h
 *
 *  Created on: 2015-06-04
 *      Author: nicholas
 */

#ifndef THROW_IF_H_
#define THROW_IF_H_
#include <string>
#include <stdexcept>

inline void throw_if(bool cond, const std::string& msg) {
    if(cond)
        throw std::runtime_error(msg);
}

#endif /* THROW_IF_H_ */
