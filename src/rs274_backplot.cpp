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
 * rs274_backplot.cpp
 *
 *  Created on: 2014-12-31
 *      Author: nicholas
 */

#include "rs274_backplot.h"
#include <cmath>
#include <cstring>
#include "Path.h"
#include <osg/Geometry>

cxxcam::Position rs274_backplot::convert(const Position& p) const
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

void rs274_backplot::interp_init()
{
	_spindle_speed = 0;
	_spindle_turning = Direction::Stop;
	_traverse_rate = 60;
}

void rs274_backplot::offset_origin(const Position& pos)
{
    program_pos = program_pos + origin_pos - pos;
    origin_pos = pos;
}


void rs274_backplot::units(Units u)
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

void rs274_backplot::rapid_rate(double rate)
{
    _traverse_rate = rate;
}

void pushBackplot(osg::Geode* geode, const std::vector<cxxcam::path::step>& steps, bool cut) {
    auto geom = new osg::Geometry();

    auto vertices = new osg::Vec3Array;
    vertices->reserve(steps.size());
    for(auto& step : steps)
    {
        auto& p = step.position;
        vertices->push_back({static_cast<float>(p.x.value()), static_cast<float>(p.y.value()), static_cast<float>(p.z.value())});
    }
    geom->setVertexArray(vertices);

    auto colors = new osg::Vec4Array;
    if(cut)
        colors->push_back({0.0f,1.0f,0.0f,1.0f});
    else
        colors->push_back({1.0f,0.0f,0.0f,1.0f});
    geom->setColorArray(colors, osg::Array::BIND_OVERALL);

    auto normals = new osg::Vec3Array;
    normals->push_back({0.0f,0.0f,1.0f});
    geom->setNormalArray(normals, osg::Array::BIND_OVERALL);

    geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP,0,vertices->size()));

    geode->addDrawable(geom);
}

void rs274_backplot::rapid(const Position& pos)
{
	auto steps = cxxcam::path::expand_linear(convert(program_pos), convert(pos), {}, 1).path;
    pushBackplot(geode, steps, false);

    program_pos = pos;
}


void rs274_backplot::feed_rate(double rate)
{
    _feed_rate = rate;
}


void rs274_backplot::feed_reference(FeedReference reference)
{
}


void rs274_backplot::motion_mode(Motion mode)
{
    _motion_mode = mode;
}


void rs274_backplot::plane(Plane pl)
{
    _active_plane = pl;
}


void rs274_backplot::cutter_radius_comp(double radius)
{
}

void rs274_backplot::cutter_radius_comp_start(Side direction)
{
}


void rs274_backplot::cutter_radius_comp_stop()
{
}

void rs274_backplot::speed_feed_sync_start()
{
}

void rs274_backplot::speed_feed_sync_stop()
{
}

void rs274_backplot::arc(double end0, double end1, double axis0, double axis1, int rotation, double end_point, double a, double b, double c)
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

	auto steps = cxxcam::path::expand_arc(convert(program_pos), convert(end), convert(center), (rotation < 0 ? cxxcam::path::ArcDirection::Clockwise : cxxcam::path::ArcDirection::CounterClockwise), plane, std::abs(rotation), {}).path;
    pushBackplot(geode, steps, true);

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


void rs274_backplot::linear(const Position& pos)
{
	auto steps = cxxcam::path::expand_linear(convert(program_pos), convert(pos), {}, 1).path;
    pushBackplot(geode, steps, true);

    program_pos = pos;
}

void rs274_backplot::probe(const Position& pos)
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


void rs274_backplot::dwell(double seconds)
{
}

void rs274_backplot::spindle_start_clockwise()
{
    _spindle_turning = ((_spindle_speed == 0) ? Direction::Stop : Direction::Clockwise);
}


void rs274_backplot::spindle_start_counterclockwise()
{
    _spindle_turning = ((_spindle_speed == 0) ? Direction::Stop : Direction::CounterClockwise);
}


void rs274_backplot::spindle_speed(double r)
{
    _spindle_speed = r;
}

void rs274_backplot::spindle_mode(double) {
}
double rs274_backplot::spindle_mode() const {
}

void rs274_backplot::spindle_stop()
{
    _spindle_turning = Direction::Stop;
}


void rs274_backplot::spindle_orient(double orientation, Direction direction)
{
}


void rs274_backplot::tool_length_offset(double length)
{
}

void rs274_backplot::tool_change(int slot)
{
    _active_slot = slot;
}


void rs274_backplot::tool_select(int i)
{
}

void rs274_backplot::axis_clamp(Axis axis)
{
}


void rs274_backplot::comment(const char *s)
{
}

void rs274_backplot::feed_override_disable()
{
}

void rs274_backplot::speed_override_disable()
{
}

void rs274_backplot::feed_override_enable()
{
}

void rs274_backplot::speed_override_enable()
{
}

void rs274_backplot::coolant_flood_off()
{
    _flood = 0;
}


void rs274_backplot::coolant_flood_on()
{
    _flood = 1;
}


void rs274_backplot::message(const char *s)
{
}

void rs274_backplot::coolant_mist_off()
{
    _mist = 0;
}


void rs274_backplot::coolant_mist_on()
{
    _mist = 1;
}


void rs274_backplot::pallet_shuttle()
{
}


void rs274_backplot::probe_off()
{
}

void rs274_backplot::probe_on()
{
}

void rs274_backplot::axis_unclamp(Axis axis)
{
}

void rs274_backplot::program_stop()
{
}

void rs274_backplot::program_optional_stop()
{
}

void rs274_backplot::program_end()
{
}

double rs274_backplot::feed_rate() const
{
    return _feed_rate;
}
bool rs274_backplot::coolant_flood() const
{
    return _flood;
}
Units rs274_backplot::units() const
{
    return _length_unit_type;
}
bool rs274_backplot::coolant_mist() const
{
    return _mist;
}
Motion rs274_backplot::motion_mode() const
{
    return _motion_mode;
}
void rs274_backplot::get_parameter_filename(char* filename, size_t max_size) const
{
    filename[0] = 0;
}
Plane rs274_backplot::plane() const
{
    return _active_plane;
}
Position rs274_backplot::current_position() const
{
	return program_pos;
}
Position rs274_backplot::probe_position() const
{
	return probe_pos;
}
double rs274_backplot::probe_value() const
{
    return 1.0;
}
bool rs274_backplot::queue_empty() const
{
    return 1;
}
double rs274_backplot::spindle_speed() const
{
    return _spindle_speed;
}
Direction rs274_backplot::spindle_state() const
{
    return _spindle_turning;
}
int rs274_backplot::tool_slot() const
{
    return _active_slot;
}
unsigned int rs274_backplot::tool_max() const
{
    return _tool_max;
}
Tool rs274_backplot::tool(int pocket) const
{
    return _tools[pocket];
}
double rs274_backplot::rapid_rate() const
{
    return _traverse_rate;
}

rs274_backplot::rs274_backplot(osg::Geode* geode)
 : geode(geode)
{
	init();
}
