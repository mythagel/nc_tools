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
 * luaconfig.cpp
 *
 *  Created on: 2015-06-02
 *      Author: nicholas
 */

#include "luaconfig.h"
#include <stdexcept>
#include <lua.hpp>

namespace lua {

config::config() {
}

bool config::get(const char* table, const char* field, std::string& value) {
    lua_getglobal(L, table);
    lua_getfield(L, -1, field);
    if (lua_isnil(L, -1)) {
        lua_pop(L, 2);
        return false;
    }
    if (!lua_isstring(L, -1)) {
        lua_pop(L, 2);
        throw std::runtime_error("Expected string value");
    }
    value = lua_tostring(L, -1);
    lua_pop(L, 2);
    return true;
}

config::~config() {
}

}
