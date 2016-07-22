/* 
 * Copyright (C) 2016  Nicholas Gill
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
 * nc_config.cpp
 *
 *  Created on: 2016-02-09
 *      Author: nicholas
 */

#include "nc_config.h"
#include <boost/filesystem.hpp>
#include <stdexcept>
#include <iostream>

namespace fs = boost::filesystem;

namespace {

std::string find_config_file(const std::string& conf) {
    if(!conf.empty())
        return conf;

    /* Search cwd then parents for nc_tools.conf */
    auto cwd = fs::current_path();
    while(!cwd.empty()) {
        auto candidate = cwd / "nc_tools.conf";
        if(exists(candidate))
            return candidate.native();
        cwd = cwd.parent_path();
    }
    return {};
}

}

nc_config::nc_config(const std::string& conf)
{
    auto config = find_config_file(conf);
    try {
        if(!config.empty() && luaL_dofile(L, config.c_str())) {
            std::string ex = lua_tostring(L, -1);
            lua_pop(L, 1);
            throw std::runtime_error(ex);
        }
    } catch(const std::exception& ex) {
        std::cerr << ex.what() << "\n";
        if(!conf.empty())
            throw;
    }
}
