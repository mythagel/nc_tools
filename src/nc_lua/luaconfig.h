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
 * luaconfig.h
 *
 *  Created on: 2015-06-02
 *      Author: nicholas
 */

#ifndef LUACONFIG_H_
#define LUACONFIG_H_
#include <string>
#include "luastate.h"

namespace lua {

/* Add functions to read value from lua table
 * at creation creates a lua state and runs config file
 * allows querying of configuration tables as
 * OPTIONAL table name, followed by value as type.
 * */
class config {
private:
    state L;
public:
    config();
    bool get(const char* table, const char* field, std::string& value);
    ~config();
};

}

#endif /* LUACONFIG_H_ */
