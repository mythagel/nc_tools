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
 * state.h
 *
 *  Created on: 2015-06-03
 *      Author: nicholas
 */

#ifndef LUASTATE_H_
#define LUASTATE_H_
#include <cstddef>
#include <lua.hpp>

namespace lua {

struct state {
    lua_State* L;
    state();
    state(const state&) = delete;
    state& operator=(const state&) = delete;
    state(state&& s);
    state& operator=(state&&);
    operator lua_State*();
    ~state();
};

size_t table_size(state& L);

}

#endif /* LUASTATE_H_ */
