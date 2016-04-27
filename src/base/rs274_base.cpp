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
 * rs274_base.cpp
 *
 *  Created on: 2015-05-26
 *      Author: nicholas
 */

#include "rs274_base.h"
#include <cmath>
#include <cstring>
#include <lua.hpp>
#include "../r6.h"
#include <sstream>

std::string str(const block_t& block)
{
    std::ostringstream s;
    auto out = [&s](char A, maybe<double> a){
        if (a) s << A << r6(*a) << ' ';
    };
    auto outi = [&s](char A, maybe<unsigned int> a){
        if (a) s << A << r6(*a) << ' ';
    };
    outi('N', block.line_number);

    for (unsigned i = 0; i < 15; ++i) {
        if (block.g_modes[i] != -1)
            out('G', static_cast<double>(block.g_modes[i])/10.0);
    }

    out('X', block.x);
    out('Y', block.y);
    out('Z', block.z);
    out('A', block.a);
    out('B', block.b);
    out('C', block.c);

    outi('H', block.h);

    out('I', block.i);
    out('J', block.j);
    out('K', block.k);

    outi('L', block.l);

    out('P', block.p);
    out('Q', block.q);
    out('R', block.r);
    out('S', block.s);

    for (unsigned i = 0; i < 10; ++i) {
        if (block.m_modes[i] != -1)
            outi('M', block.m_modes[i]);
    }

    outi('T', block.t);
    out('D', block.d);
    out('F', block.f);

    if (block.comment[0]) s << "(" << block.comment << ") ";

    return s.str();
}

rs274_base::rs274_base(const std::string& conf)
 : config(conf)
{
	init();
}
rs274_base::~rs274_base()
{
}

cxxcam::Position rs274_base::convert(const Position& p) const
{
    using namespace cxxcam;
    using namespace cxxcam::units;
    cxxcam::Position pos;
    if(_length_unit_type == Units::Metric)
    {
        pos.X = length{p.x * millimeters};
        pos.Y = length{p.y * millimeters};
        pos.Z = length{p.z * millimeters};
    }
    else
    {
        pos.X = length{p.x * inches};
        pos.Y = length{p.y * inches};
        pos.Z = length{p.z * inches};
    }
    pos.A = plane_angle{p.a * degrees};
    pos.B = plane_angle{p.b * degrees};
    pos.C = plane_angle{p.c * degrees};
    return pos;
}

double rs274_base::spindle_delta_theta(const cxxcam::units::length& motion_length)
{
    using namespace cxxcam;
    static const double PI = 3.14159265358979323846;
    auto feed_rate = [&] {
        if(_length_unit_type == Units::Metric) {
            return units::velocity{ _feed_rate * units::millimeters_per_minute };
        } else {
            return units::velocity{ _feed_rate * units::inches_per_minute };
        }
    }();

    auto time = units::time{ motion_length / feed_rate };
    auto spindle_rads_s = _spindle_speed * 60 * 2 * PI;
    switch (_spindle_turning) {
        case Direction::CounterClockwise:
            spindle_rads_s *= -1;
            break;
        case Direction::Clockwise:
            spindle_rads_s *= 1;
            break;
        case Direction::Stop:
            spindle_rads_s = 0;
            break;
    }

    auto theta = spindle_rads_s * time.value();
    _spindle_theta += theta;
    _spindle_theta = std::fmod(_spindle_theta, 2*PI);
    return theta;
}

void rs274_base::interp_init()
{
	_spindle_speed = 0;
	_spindle_turning = Direction::Stop;
	_traverse_rate = 60;
    _spindle_theta = 0;
}

void rs274_base::offset_origin(const Position& pos)
{
    program_pos = program_pos + origin_pos - pos;
    origin_pos = pos;
}


void rs274_base::units(Units u)
{
    if (u == Units::Imperial)
    {
        if (_length_unit_type == Units::Metric)
        {
            _length_unit_type = Units::Imperial;
            origin_pos.x = (origin_pos.x / 25.4);
            origin_pos.y = (origin_pos.y / 25.4);
            origin_pos.z = (origin_pos.z / 25.4);
            program_pos.x = (program_pos.x / 25.4);
            program_pos.y = (program_pos.y / 25.4);
            program_pos.z = (program_pos.z / 25.4);
        }
    }
    else if (u == Units::Metric)
    {
        if (_length_unit_type == Units::Imperial)
        {
            _length_unit_type = Units::Metric;
            origin_pos.x = (origin_pos.x * 25.4);
            origin_pos.y = (origin_pos.y * 25.4);
            origin_pos.z = (origin_pos.z * 25.4);
            program_pos.x = (program_pos.x * 25.4);
            program_pos.y = (program_pos.y * 25.4);
            program_pos.z = (program_pos.z * 25.4);
        }
    }
}

void rs274_base::rapid_rate(double rate)
{
    _traverse_rate = rate;
}

void rs274_base::rapid(const Position& pos)
{
    _rapid(pos);
    program_pos = pos;
}


void rs274_base::feed_rate(double rate)
{
    _feed_rate = rate;
}


void rs274_base::feed_reference(FeedReference)
{
}


void rs274_base::motion_mode(Motion mode)
{
    _motion_mode = mode;
}


void rs274_base::plane(Plane pl)
{
    _active_plane = pl;
}


void rs274_base::cutter_radius_comp(double)
{
}

void rs274_base::cutter_radius_comp_start(Side)
{
}


void rs274_base::cutter_radius_comp_stop()
{
}

void rs274_base::speed_feed_sync_start()
{
}

void rs274_base::speed_feed_sync_stop()
{
}

void rs274_base::arc(double end0, double end1, double axis0, double axis1, int rotation, double end_point, double a, double b, double c)
{
    Position end;
    Position center;
    cxxcam::math::vector_3 plane;

    switch(_active_plane) {
        case Plane::XY:
            end.x = end0;
            end.y = end1;
            end.z = end_point;
            center.x = axis0;
            center.y = axis1;
            plane.z = 1;
            break;
        case Plane::YZ:
            end.x = end_point;
            end.y = end0;
            end.z = end1;
            center.y = axis0;
            center.z = axis1;
            plane.x = 1;
            break;
        case Plane::XZ:
            end.x = end1;
            end.y = end_point;
            end.z = end0;
            center.x = axis1;
            center.z = axis0;
            plane.y = 1;
            break;
    }
    end.a = a;
    end.b = b;
    end.c = c;

    _arc(end, center, plane, rotation);

    if (_active_plane == Plane::XY)
    {
        program_pos.x = end0;
        program_pos.y = end1;
        program_pos.z = end_point;
    }
    else if (_active_plane == Plane::YZ)
    {
        program_pos.x = end_point;
        program_pos.y = end0;
        program_pos.z = end1;
    }
    else
    {
        program_pos.x = end1;
        program_pos.y = end_point;
        program_pos.z = end0;
    }
    program_pos.a = a;
    program_pos.b = b;
    program_pos.c = c;
}


void rs274_base::linear(const Position& pos)
{
    _linear(pos);
    program_pos = pos;
}

void rs274_base::probe(const Position& pos)
{
    double distance;
    double dx, dy, dz;
    double backoff;

    dx = (program_pos.x - pos.x);
    dy = (program_pos.y - pos.y);
    dz = (program_pos.z - pos.z);
    distance = sqrt((dx * dx) + (dy * dy) + (dz * dz));

    probe_pos = pos;
    if (distance != 0)
    {
        backoff = ((_length_unit_type == Units::Metric) ? 0.254 : 0.01);
        program_pos.x = (pos.x + (backoff * (dx / distance)));
        program_pos.y = (pos.y + (backoff * (dy / distance)));
        program_pos.z = (pos.z + (backoff * (dz / distance)));
    }
    program_pos.a = pos.a;
    program_pos.b = pos.b;
    program_pos.c = pos.c;
}


void rs274_base::dwell(double)
{
}

void rs274_base::spindle_start_clockwise()
{
    _spindle_turning = ((_spindle_speed == 0) ? Direction::Stop : Direction::Clockwise);
}


void rs274_base::spindle_start_counterclockwise()
{
    _spindle_turning = ((_spindle_speed == 0) ? Direction::Stop : Direction::CounterClockwise);
}


void rs274_base::spindle_speed(double r)
{
    _spindle_speed = r;
}

void rs274_base::spindle_mode(double) {
}
double rs274_base::spindle_mode() const {
    return 0.0;
}

void rs274_base::spindle_stop()
{
    _spindle_turning = Direction::Stop;
}


void rs274_base::spindle_orient(double, Direction)
{
}


void rs274_base::tool_length_offset(double)
{
}

void rs274_base::tool_change(int slot)
{
    _active_slot = slot;
}


void rs274_base::tool_select(int)
{
}

void rs274_base::axis_clamp(Axis)
{
}


void rs274_base::comment(const char *)
{
}

void rs274_base::feed_override_disable()
{
}

void rs274_base::speed_override_disable()
{
}

void rs274_base::feed_override_enable()
{
}

void rs274_base::speed_override_enable()
{
}

void rs274_base::coolant_flood_off()
{
    _flood = 0;
}


void rs274_base::coolant_flood_on()
{
    _flood = 1;
}


void rs274_base::message(const char *)
{
}

void rs274_base::coolant_mist_off()
{
    _mist = 0;
}


void rs274_base::coolant_mist_on()
{
    _mist = 1;
}


void rs274_base::pallet_shuttle()
{
}


void rs274_base::probe_off()
{
}

void rs274_base::probe_on()
{
}

void rs274_base::axis_unclamp(Axis)
{
}

void rs274_base::program_stop()
{
}

void rs274_base::program_optional_stop()
{
}

void rs274_base::program_end()
{
}

double rs274_base::feed_rate() const
{
    return _feed_rate;
}
bool rs274_base::coolant_flood() const
{
    return _flood;
}
Units rs274_base::units() const
{
    return _length_unit_type;
}
bool rs274_base::coolant_mist() const
{
    return _mist;
}
Motion rs274_base::motion_mode() const
{
    return _motion_mode;
}
void rs274_base::get_parameter_filename(char* filename, size_t) const
{
    filename[0] = 0;
}
Plane rs274_base::plane() const
{
    return _active_plane;
}
Position rs274_base::current_position() const
{
	return program_pos;
}
Position rs274_base::probe_position() const
{
	return probe_pos;
}
double rs274_base::probe_value() const
{
    return 1.0;
}
bool rs274_base::queue_empty() const
{
    return 1;
}
double rs274_base::spindle_speed() const
{
    return _spindle_speed;
}
Direction rs274_base::spindle_state() const
{
    return _spindle_turning;
}
int rs274_base::tool_slot() const
{
    return _active_slot;
}
unsigned int rs274_base::tool_max() const
{
    return CANON_TOOL_MAX;
}
Tool rs274_base::tool(int pocket) const
{
    auto& L = config.state();

    lua_getglobal(L, "tool_table");
    if (!lua_istable(L, -1)) {
        lua_pop(L, 1);
        return {};
    }

    lua_pushinteger(L, pocket);
    lua_gettable(L, -2);
    if (!lua_istable(L, -1)) {
        lua_pop(L, 2);
        return {};
    }
    
    Tool t;
    t.id = pocket;
    
    lua_getfield(L, -1, "length");
    if(lua_isnumber(L, -1))
        t.length = lua_tonumber(L, -1);
    lua_pop(L, 1);

    lua_getfield(L, -1, "diameter");
    if(lua_isnumber(L, -1))
        t.diameter = lua_tonumber(L, -1);
    lua_pop(L, 1);

    lua_pop(L, 1);
    return t;
}
double rs274_base::rapid_rate() const
{
    return _traverse_rate;
}

void rs274_base::block_end(const block_t&)
{
}

