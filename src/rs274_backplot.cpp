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

void rs274_backplot::interp_init()
{
	_parameter_file_name[0] = 0;
	_spindle_speed = 0;
	_spindle_turning = Direction::Stop;
	_traverse_rate = 60;
}

void rs274_backplot::print_nc_line_number()
{
    char text[256];
    int k;
    int m;

    line_text(text, 256);
    for (k = 0;
        ((k < 256) and
        ((text[k] == '\t') or (text[k] == ' ') or (text[k] == '/')));
        k++);
    if ((k < 256) and ((text[k] == 'n') or (text[k] == 'N')))
    {
        fputc('N', stdout);
        for (k++, m = 0;
            ((k < 256) and (text[k] >= '0') and (text[k] <= '9'));
            k++, m++)
        fputc(text[k], stdout);
        for (; m < 6; m++)
            fputc(' ', stdout);
    }
    else if (k < 256)
        fprintf(stdout, "N..... ");
}

#define PRINT(format, ...) do \
{ \
	fprintf(stdout, "%5d ", _line_number++); \
	print_nc_line_number(); \
	fprintf(stdout, format, ##__VA_ARGS__); \
} while(0)

void rs274_backplot::offset_origin(const Position& pos)
{
    PRINT("SET_ORIGIN_OFFSETS(%.4f, %.4f, %.4f, %.4f, %.4f, %.4f)\n", pos.x, pos.y, pos.z, pos.a, pos.b, pos.c);

    program_pos = program_pos + origin_pos - pos;
    origin_pos = pos;
}


void rs274_backplot::units(Units u)
{
    if (u == Units::Imperial)
    {
        PRINT("USE_LENGTH_UNITS(CANON_UNITS_INCHES)\n");
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
        PRINT("USE_LENGTH_UNITS(CANON_UNITS_MM)\n");
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
    else
        PRINT("USE_LENGTH_UNITS(UNKNOWN)\n");
}

void rs274_backplot::rapid_rate(double rate)
{
    PRINT("SET_TRAVERSE_RATE(%.4f)\n", rate);
    _traverse_rate = rate;
}


void rs274_backplot::rapid(const Position& pos)
{
    PRINT("STRAIGHT_TRAVERSE(%.4f, %.4f, %.4f, %.4f, %.4f, %.4f)\n", pos.x, pos.y, pos.z, pos.a, pos.b, pos.c);
    program_pos = pos;
}


   /* Machining Attributes */
void rs274_backplot::feed_rate(double rate)
{
    PRINT("SET_FEED_RATE(%.4f)\n", rate);
    _feed_rate = rate;
}


void rs274_backplot::feed_reference(FeedReference reference)
{
    PRINT("SET_FEED_REFERENCE(%s)\n", (reference == FeedReference::Workpiece) ? "CANON_WORKPIECE" : "CANON_XYZ");
}


void rs274_backplot::motion_mode(Motion mode)
{
    if (mode == Motion::Exact_Stop)
    {
        PRINT("SET_MOTION_CONTROL_MODE(CANON_EXACT_STOP)\n");
        _motion_mode = Motion::Exact_Stop;
    }
    else if (mode == Motion::Exact_Path)
    {
        PRINT("SET_MOTION_CONTROL_MODE(CANON_EXACT_PATH)\n");
        _motion_mode = Motion::Exact_Path;
    }
    else if (mode == Motion::Continuous)
    {
        PRINT("SET_MOTION_CONTROL_MODE(CANON_CONTINUOUS)\n");
        _motion_mode = Motion::Continuous;
    }
    else
        PRINT("SET_MOTION_CONTROL_MODE(UNKNOWN)\n");
}


void rs274_backplot::plane(Plane pl)
{
    PRINT("SELECT_PLANE(CANON_PLANE_%s)\n",
        ((pl == Plane::XY) ? "XY" :
		(pl == Plane::YZ) ? "YZ" :
		(pl == Plane::XZ) ? "XZ" : "UNKNOWN"));
    _active_plane = pl;
}


void rs274_backplot::cutter_radius_comp(double radius)
{
	PRINT("SET_CUTTER_RADIUS_COMPENSATION(%.4f)\n", radius);
}

void rs274_backplot::cutter_radius_comp_start(Side direction)
{
    PRINT("START_CUTTER_RADIUS_COMPENSATION(%s)\n",
        (direction == Side::Left)  ? "LEFT"  :
		(direction == Side::Right) ? "RIGHT" : "UNKNOWN");
}


void rs274_backplot::cutter_radius_comp_stop()
{
	PRINT ("STOP_CUTTER_RADIUS_COMPENSATION()\n");
}

void rs274_backplot::speed_feed_sync_start()
{
	PRINT ("START_SPEED_FEED_SYNCH()\n");
}

void rs274_backplot::speed_feed_sync_stop()
{
	PRINT ("STOP_SPEED_FEED_SYNCH()\n");
}

   /* Machining Functions */

void rs274_backplot::arc(double end0, double end1, double axis0, double axis1, int rotation, double end_point, double a, double b, double c)
{
    PRINT("ARC_FEED(%.4f, %.4f, %.4f, %.4f, %d, %.4f, %.4f, %.4f, %.4f)\n", end0, end1, axis0, axis1, rotation, end_point, a, b, c);
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
    else                                          /* if (_active_plane == CANON_PLANE_XZ) */
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
    PRINT("STRAIGHT_FEED(%.4f, %.4f, %.4f, %.4f, %.4f, %.4f)\n", pos.x, pos.y, pos.z, pos.a, pos.b, pos.c);
    program_pos = pos;
}


   /* This models backing the probe off 0.01 inch or 0.254 mm from the probe
   point towards the previous location after the probing, if the probe
   point is not the same as the previous point -- which it should not be. */

void rs274_backplot::probe(const Position& pos)
{
    double distance;
    double dx, dy, dz;
    double backoff;

    dx = (program_pos.x - pos.x);
    dy = (program_pos.y - pos.y);
    dz = (program_pos.z - pos.z);
    distance = sqrt((dx * dx) + (dy * dy) + (dz * dz));

    PRINT("STRAIGHT_PROBE(%.4f, %.4f, %.4f, %.4f, %.4f, %.4f)\n", pos.x, pos.y, pos.z, pos.a, pos.b, pos.c);
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
	PRINT("DWELL(%.4f)\n", seconds);
}

   /* Spindle Functions */

void rs274_backplot::spindle_start_clockwise()
{
    PRINT("START_SPINDLE_CLOCKWISE()\n");
    _spindle_turning = ((_spindle_speed == 0) ? Direction::Stop : Direction::Clockwise);
}


void rs274_backplot::spindle_start_counterclockwise()
{
    PRINT("START_SPINDLE_COUNTERCLOCKWISE()\n");
    _spindle_turning = ((_spindle_speed == 0) ? Direction::Stop : Direction::CounterClockwise);
}


void rs274_backplot::spindle_speed(double r)
{
    PRINT("SET_SPINDLE_SPEED(%.4f)\n", r);
    _spindle_speed = r;
}

void rs274_backplot::spindle_mode(double) {
}
double rs274_backplot::spindle_mode() const {
}

void rs274_backplot::spindle_stop()
{
    PRINT("STOP_SPINDLE_TURNING()\n");
    _spindle_turning = Direction::Stop;
}


void rs274_backplot::spindle_orient(double orientation, Direction direction)
{
    PRINT("ORIENT_SPINDLE(%.4f, %s)\n", orientation, (direction == Direction::Clockwise) ? "CANON_CLOCKWISE" : "CANON_COUNTERCLOCKWISE");
}


   /* Tool Functions */

void rs274_backplot::tool_length_offset(double length)
{
	PRINT("USE_TOOL_LENGTH_OFFSET(%.4f)\n", length);
}

void rs274_backplot::tool_change(int slot)
{
    PRINT("CHANGE_TOOL(%d)\n", slot);
    _active_slot = slot;
}


void rs274_backplot::tool_select(int i)
{
	PRINT("SELECT_TOOL(%d)\n", i);
}

   /* Misc Functions */

void rs274_backplot::axis_clamp(Axis axis)
{
    PRINT("CLAMP_AXIS(%s)\n",
        (axis == Axis::X) ? "CANON_AXIS_X" :
		(axis == Axis::Y) ? "CANON_AXIS_Y" :
		(axis == Axis::Z) ? "CANON_AXIS_Z" :
		(axis == Axis::A) ? "CANON_AXIS_A" :
		(axis == Axis::B) ? "CANON_AXIS_B" :
		(axis == Axis::C) ? "CANON_AXIS_C" : "UNKNOWN");
}


void rs274_backplot::comment(const char *s)
{PRINT("COMMENT(\"%s\")\n", s);}

void rs274_backplot::feed_override_disable()
{PRINT("DISABLE_FEED_OVERRIDE()\n");}

void rs274_backplot::speed_override_disable()
{PRINT("DISABLE_SPEED_OVERRIDE()\n");}

void rs274_backplot::feed_override_enable()
{PRINT("ENABLE_FEED_OVERRIDE()\n");}

void rs274_backplot::speed_override_enable()
{PRINT("ENABLE_SPEED_OVERRIDE()\n");}

void rs274_backplot::coolant_flood_off()
{
    PRINT("FLOOD_OFF()\n");
    _flood = 0;
}


void rs274_backplot::coolant_flood_on()
{
    PRINT("FLOOD_ON()\n");
    _flood = 1;
}


void rs274_backplot::message(const char *s)
{
	PRINT("MESSAGE(\"%s\")\n", s);
}

void rs274_backplot::coolant_mist_off()
{
    PRINT("MIST_OFF()\n");
    _mist = 0;
}


void rs274_backplot::coolant_mist_on()
{
    PRINT("MIST_ON()\n");
    _mist = 1;
}


void rs274_backplot::pallet_shuttle()
{PRINT("PALLET_SHUTTLE()\n");}


void rs274_backplot::probe_off()
{PRINT("TURN_PROBE_OFF()\n");}

void rs274_backplot::probe_on()
{PRINT("TURN_PROBE_ON()\n");}

void rs274_backplot::axis_unclamp(Axis axis)
{
    PRINT("UNCLAMP_AXIS(%s)\n",
        (axis == Axis::X) ? "CANON_AXIS_X" :
    (axis == Axis::Y) ? "CANON_AXIS_Y" :
    (axis == Axis::Z) ? "CANON_AXIS_Z" :
    (axis == Axis::A) ? "CANON_AXIS_A" :
    (axis == Axis::B) ? "CANON_AXIS_B" :
    (axis == Axis::C) ? "CANON_AXIS_C" : "UNKNOWN");
}


   /* Program Functions */

void rs274_backplot::program_stop()
{PRINT("PROGRAM_STOP()\n");}

void rs274_backplot::program_optional_stop()
{PRINT("OPTIONAL_PROGRAM_STOP()\n");}

void rs274_backplot::program_end()
{PRINT("PROGRAM_END()\n");}

   /*************************************************************************/

   /* Canonical "Give me information" functions

   In general, returned values are valid only if any canonical do it commands
   that may have been called for have been executed to completion. If a function
   returns a valid value regardless of execution, that is noted in the comments
   below.

   */

   /* The interpreter is not using this function
   // Returns the system angular unit factor, in units / degree
   extern double GET_EXTERNAL_ANGLE_UNIT_FACTOR()
   {
   return 1;
   }
   */

   /* Returns the system feed rate */
double rs274_backplot::feed_rate() const
{
    return _feed_rate;
}


   /* Returns the system flood coolant setting zero = off, non-zero = on */
bool rs274_backplot::coolant_flood() const
{
    return _flood;
}


   /* Returns the system length unit type */
Units rs274_backplot::units() const
{
    return _length_unit_type;
}


   /* Returns the system mist coolant setting zero = off, non-zero = on */
bool rs274_backplot::coolant_mist() const
{
    return _mist;
}


   // Returns the current motion control mode
Motion rs274_backplot::motion_mode() const
{
    return _motion_mode;
}


void rs274_backplot::get_parameter_filename(char* filename, size_t max_size) const
{
    if (strlen(_parameter_file_name) < max_size)
        strcpy(filename, _parameter_file_name);
    else
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

   /* Returns the value for any analog non-contact probing. */
   /* This is a dummy of a dummy, returning a useless value. */
   /* It is not expected this will ever be called. */
double rs274_backplot::probe_value() const
{
    return 1.0;
}


   /* Returns zero if queue is not empty, non-zero if the queue is empty */
   /* In the stand-alone interpreter, there is no queue, so it is always empty */
bool rs274_backplot::queue_empty() const
{
    return 1;
}


   /* Returns the system value for spindle speed in rpm */
double rs274_backplot::spindle_speed() const
{
    return _spindle_speed;
}


   /* Returns the system value for direction of spindle turning */
Direction rs274_backplot::spindle_state() const
{
    return _spindle_turning;
}


   /* Returns the system value for the carousel slot in which the tool
   currently in the spindle belongs. Return value zero means there is no
   tool in the spindle. */
int rs274_backplot::tool_slot() const
{
    return _active_slot;
}


   /* Returns maximum number of tools */
unsigned int rs274_backplot::tool_max() const
{
    return _tool_max;
}


   /* Returns the CANON_TOOL_TABLE structure associated with the tool
      in the given pocket */
Tool rs274_backplot::tool(int pocket) const
{
    return _tools[pocket];
}


   /* Returns the system traverse rate */
double rs274_backplot::rapid_rate() const
{
    return _traverse_rate;
}

rs274_backplot::rs274_backplot(osg::Geode* geode)
 : geode(geode)
{
	init();
}
