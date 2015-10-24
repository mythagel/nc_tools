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
 * state.cpp
 *
 *  Created on: 2015-06-03
 *      Author: nicholas
 */

#include "state.h"
#include <lua.hpp>
#include <utility>
#include "../throw_if.h"

namespace lua {

state::state()
 : L(luaL_newstate()) {
    throw_if(!L, "Unable to create lua state");
}
state::state(state&& s)
 : L(nullptr) {
    using std::swap;
    swap(L, s.L);
}
state& state::operator=(state&& s) {
    using std::swap;
    swap(L, s.L);
    return *this;
}
state::operator lua_State*() {
    return L;
}
state::~state() {
    if(L) lua_close(L);
}

size_t table_size(state& L) {
    size_t count = 0;

    auto t = lua_gettop(L);
    lua_pushnil(L);
    while(lua_next(L, t)) {
        ++count;
        lua_pop(L, 1);
    }
    lua_pop(L, 1);
    return count;
    
}

}
