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
 * machine_config.cpp
 *
 *  Created on: 2016-04-30
 *      Author: nicholas
 */

#include "machine_config.h"
#include "../throw_if.h"
#include <cstring>

namespace po = boost::program_options;

namespace {

template <typename Fn>
class guard {
private:
    Fn fn;
    bool armed;
public:
    guard(Fn fn) : fn(fn), armed(true) {}
    void disarm() { armed = false; }
    ~guard() { if (armed) fn(); }
};
template<typename Fn> guard<Fn> make_guard(Fn fn) {
    return { fn };
}

void get_machine(lua::state& L, const std::string& name) {
    lua_getglobal(L, "machine");
    auto pop_global = make_guard([&]{ lua_pop(L, 1); });
    throw_if (!lua_istable(L, -1), "machine config missing / incorrect");


    lua_getfield(L, -1, name.c_str());
    auto pop_machine_field = make_guard([&]{ lua_pop(L, 1); });
    throw_if(!lua_istable(L, -1), "named machine config missing");

    lua_remove(L, -2);
    pop_global.disarm();
    pop_machine_field.disarm();
}

}

namespace machine_config {

po::options_description base_options() {
    po::options_description options("base options");
    options.add_options()
        ("config", po::value<std::string>()->default_value(""), "Configuration file")
        ("machine", po::value<std::string>(), "Machine configuration")
    ;

    return options;
}

std::string default_machine(nc_config& config) {
    auto& L = config.state();

    lua_getglobal(L, "default");
    auto pop_global = make_guard([&]{ lua_pop(L, 1); });
    if (lua_isnil(L, -1)) return "__default__";
    throw_if (!lua_istable(L, -1), "default config missing / incorrect");

    lua_getfield(L, -1, "machine");
    auto pop_machine_field = make_guard([&]{ lua_pop(L, 1); });
    throw_if(!lua_isstring(L, -1), "machine default string missing / incorrect");

    return lua_tostring(L, -1);
}

units default_units(nc_config& config) {
    auto& L = config.state();

    lua_getglobal(L, "default");
    auto pop_global = make_guard([&]{ lua_pop(L, 1); });
    if (lua_isnil(L, -1)) return units::metric;
    throw_if (!lua_istable(L, -1), "default config missing / incorrect");

    lua_getfield(L, -1, "units");
    auto pop_units_field = make_guard([&]{ lua_pop(L, 1); });
    throw_if(!lua_isstring(L, -1), "units string missing / incorrect");

    if(strcmp(lua_tostring(L, -1), "metric") == 0)
        return units::metric;
    if(strcmp(lua_tostring(L, -1), "imperial") == 0)
        return units::imperial;

    throw std::runtime_error("unrecognised units");
}
units machine_units(nc_config& config, const std::string& machine) {
    auto& L = config.state();

    if (machine == "__default__")
        return units::metric;

    get_machine(L, machine);
    auto pop_machine_field = make_guard([&]{ lua_pop(L, 1); });

    lua_getfield(L, -1, "units");
    auto pop_units_field = make_guard([&]{ lua_pop(L, 1); });

    if (!lua_isnil(L, -1)) {
        throw_if(!lua_isstring(L, -1), "units string missing / incorrect");

        if(strcmp(lua_tostring(L, -1), "metric") == 0)
            return units::metric;
        if(strcmp(lua_tostring(L, -1), "imperial") == 0)
            return units::imperial;

        throw std::runtime_error("unrecognised machine type");
    } else {
        return default_units(config);
    }
}

machine_type get_machine_type(nc_config& config, const std::string& machine) {
    auto& L = config.state();

    if (machine == "__default__")
        return machine_type::mill;

    get_machine(L, machine);
    auto pop_machine_field = make_guard([&]{ lua_pop(L, 1); });

    lua_getfield(L, -1, "type");
    auto pop_type_field = make_guard([&]{ lua_pop(L, 1); });
    throw_if(!lua_isstring(L, -1), "machine type string missing / incorrect");

    
    if(strcmp(lua_tostring(L, -1), "mill") == 0)
        return machine_type::mill;
    if(strcmp(lua_tostring(L, -1), "lathe") == 0)
        return machine_type::lathe;

    throw std::runtime_error("unrecognised machine type");
}

bool get_tool(nc_config& config, unsigned id, const std::string& machine, mill_tool& tool) {
    auto& L = config.state();

    if (machine == "__default__")
        return false;

    get_machine(L, machine);
    auto pop_machine_field = make_guard([&]{ lua_pop(L, 1); });

    lua_getfield(L, -1, "tool_table");
    auto pop_tool_table = make_guard([&]{ lua_pop(L, 1); });
    throw_if (!lua_istable(L, -1), "tool_table missing / incorrect");

    lua_pushinteger(L, id);
    lua_gettable(L, -2);
    auto pop_tool_entry = make_guard([&]{ lua_pop(L, 1); });
    if (lua_isnil(L, -1)) return false;
    throw_if (!lua_istable(L, -1), "tool entry incorrect");
    
    lua_getfield(L, -1, "name");
    if(lua_isstring(L, -1))
        tool.name = lua_tostring(L, -1);
    lua_pop(L, 1);

    lua_getfield(L, -1, "length");
    if(lua_isnumber(L, -1))
        tool.length = lua_tonumber(L, -1);
    lua_pop(L, 1);

    lua_getfield(L, -1, "diameter");
    if(lua_isnumber(L, -1))
        tool.diameter = lua_tonumber(L, -1);
    lua_pop(L, 1);

    lua_getfield(L, -1, "flutes");
    if(lua_isnumber(L, -1))
        tool.flutes = lua_tonumber(L, -1);
    lua_pop(L, 1);

    lua_getfield(L, -1, "flute_length");
    if(lua_isnumber(L, -1))
        tool.flute_length = lua_tonumber(L, -1);
    lua_pop(L, 1);

    lua_getfield(L, -1, "shank_diameter");
    if(lua_isnumber(L, -1))
        tool.shank_diameter = lua_tonumber(L, -1);
    lua_pop(L, 1);

    return true;
}

bool get_tool(nc_config& config, unsigned id, const std::string& machine, lathe_tool& tool) {
    auto& L = config.state();
    get_machine(L, machine);
    auto pop_machine_field = make_guard([&]{ lua_pop(L, 1); });

    return false;
}

}
