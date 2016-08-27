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
 * rs274_model.cpp
 *
 *  Created on: 2015-05-22
 *      Author: nicholas
 */

#include "rs274_model.h"
#include <cmath>
#include <cstring>
#include "Path.h"
#include "Simulation.h"
#include <fstream>
#include "throw_if.h"
#include "geom/primitives.h"
#include "fold_adjacent.h"
#include "geom/ops.h"
#include <thread>
#include <future>
#include <iterator>
#include <algorithm>
#include "base/machine_config.h"

#include "geom/io.h"

#include <iostream>

vecTriMesh::vecTriMesh() {
    set();
}
vecTriMesh::vecTriMesh(const vecTriMesh& v)
 : _triangles(v._triangles), _vertices(v._vertices) {
    set();
}
vecTriMesh& vecTriMesh::operator=(const vecTriMesh& v) {
    _triangles = v._triangles;
    _vertices = v._vertices;
    set();
    return *this;
}
void vecTriMesh::set() {
    n_triangles = _triangles.size() / 3;
    n_vertices = _vertices.size();
    triangles = _triangles.data();
    vertices = _vertices.data();
}

namespace {

unsigned int hardware_concurrency() {
    auto cores = std::thread::hardware_concurrency();
    if(!cores) cores = 4;
    return cores;
}

static void corkTriMesh2CorkMesh(
    const vecTriMesh& in,
    CorkMesh *mesh_out
) {
    RawCorkMesh raw;
    raw.vertices.resize(in.n_vertices);
    raw.triangles.resize(in.n_triangles);
    if(in.n_vertices == 0 || in.n_triangles == 0) {
        CORK_ERROR("empty mesh input to Cork routine.");
        *mesh_out = CorkMesh(raw);
        return;
    }
    
    uint max_ref_idx = 0;
    for(uint i=0; i<in.n_triangles; i++) {
        raw.triangles[i].a = in.triangles[3*i+0];
        raw.triangles[i].b = in.triangles[3*i+1];
        raw.triangles[i].c = in.triangles[3*i+2];
        max_ref_idx = std::max(
                        std::max(max_ref_idx,
                                 in.triangles[3*i+0]),
                        std::max(in.triangles[3*i+1],
                                 in.triangles[3*i+2])
                      );
    }
    if(max_ref_idx > in.n_vertices) {
        CORK_ERROR("mesh input to Cork routine has an out of range reference "
              "to a vertex.");
        raw.vertices.clear();
        raw.triangles.clear();
        *mesh_out = CorkMesh(raw);
        return;
    }
    
    for(uint i=0; i<in.n_vertices; i++) {
        raw.vertices[i].pos.x = in.vertices[3*i+0];
        raw.vertices[i].pos.y = in.vertices[3*i+1];
        raw.vertices[i].pos.z = in.vertices[3*i+2];
    }
    
    *mesh_out = CorkMesh(raw);
}
CorkMesh to_mesh(const geom::polyhedron_t& poly) {
    CorkMesh ret;
    vecTriMesh mesh;

    to_object(poly,
        [&](double x, double y, double z) {
            mesh._vertices.push_back(x);
            mesh._vertices.push_back(y);
            mesh._vertices.push_back(z);
        },
        [&](std::size_t t0, std::size_t t1, std::size_t t2) {
            mesh._triangles.push_back(t0);
            mesh._triangles.push_back(t1);
            mesh._triangles.push_back(t2);
        }
    );

    mesh.set();
    corkTriMesh2CorkMesh(mesh, &ret);
    return ret;
}

}

/*
 * vector of future polyhedrons
 * queue of packaged_tasks
 * pool pulls tasks from queue
 * ...
 * */
CorkMesh parallel_fold_toolpath(std::vector<CorkMesh> tool_motion);
std::vector<CorkMesh> parallel_fold_toolpath(unsigned int n, std::vector<CorkMesh> tool_motion);

void rs274_model::_rapid(const Position& pos) {
    using namespace cxxcam;

	auto length = path::length_linear(convert(program_pos), convert(pos));
    auto spindle_delta = spindle_delta_theta(length);
    apply_spindle_delta(spindle_delta);
}

void rs274_model::_arc(const Position& end, const Position& center, const cxxcam::math::vector_3& plane, int rotation) {
    using namespace cxxcam;

	auto length = path::length_arc(convert(program_pos), convert(end), convert(center), (rotation < 0 ? path::ArcDirection::Clockwise : path::ArcDirection::CounterClockwise), plane, std::abs(rotation));
    auto spindle_theta = _spindle_theta;
    auto spindle_delta = spindle_delta_theta(length);
    apply_spindle_delta(spindle_delta);
    auto spindle_steps = (spindle_delta / (2*PI)) * _steps_per_revolution;
    auto spindle_step = spindle_delta / spindle_steps;

	auto steps = path::expand_arc(convert(program_pos), convert(end), convert(center), (rotation < 0 ? path::ArcDirection::Clockwise : path::ArcDirection::CounterClockwise), plane, std::abs(rotation), {}, _lathe ? spindle_steps : 1).path;

    fold_adjacent(std::begin(steps), std::end(steps), std::back_inserter(_toolpath), 
		[&](const path::step& s0, const path::step& s1) -> CorkMesh
		{
            if (_lathe) {
                auto tp = simulation::sweep_lathe_tool(_tool, s0, s1, units::plane_angle(spindle_theta * units::radians));
                spindle_theta += spindle_step;
                return to_mesh(tp);
            } else {
                return to_mesh(simulation::sweep_tool(_tool, s0, s1));
            }
		});
    if(_toolpath.size() >= 5 * hardware_concurrency())
        _toolpath = parallel_fold_toolpath(hardware_concurrency(), _toolpath);
}


void rs274_model::_linear(const Position& pos) {
    using namespace cxxcam;

	auto length = path::length_linear(convert(program_pos), convert(pos));
    auto spindle_theta = _spindle_theta;
    auto spindle_delta = spindle_delta_theta(length);
    apply_spindle_delta(spindle_delta);
    auto spindle_steps = (spindle_delta / (2*PI)) * _steps_per_revolution;
    auto spindle_step = spindle_delta / spindle_steps;

	auto steps = path::expand_linear(convert(program_pos), convert(pos), {}, _lathe ? spindle_steps : -1).path;

    fold_adjacent(std::begin(steps), std::end(steps), std::back_inserter(_toolpath), 
		[&](const path::step& s0, const path::step& s1) -> CorkMesh
		{
            if (_lathe) {
                auto tp = simulation::sweep_lathe_tool(_tool, s0, s1, units::plane_angle(spindle_theta * units::radians));
                spindle_theta += spindle_step;
                return to_mesh(tp);
            } else {
			    return to_mesh(simulation::sweep_tool(_tool, s0, s1));
            }
		});
    if(_toolpath.size() >= 5 * hardware_concurrency())
        _toolpath = parallel_fold_toolpath(hardware_concurrency(), _toolpath);
}
/* abstract out tool defs from models + add drill model where 'flutes' is tapered tip
 * */
void rs274_model::tool_change(int slot) {
    using namespace machine_config;

    if (_lathe) {
        lathe_tool t;
        get_tool(config, slot, machine_id, t);
        // TODO
    } else {
        mill_tool t;
        get_tool(config, slot, machine_id, t);
        auto shank = geom::make_cone( {0, 0, t.length}, {0, 0, t.flute_length}, t.shank_diameter/2, t.shank_diameter/2, 32);
        auto flutes = geom::make_cone( {0, 0, t.flute_length}, {0, 0, 0}, t.diameter/2, t.diameter/2, 32);
        //_tool = shank + flutes;
        _tool = flutes;
    }
}

void rs274_model::dwell(double /*seconds*/) {
    // TODO update spindle theta based on dwell time
}

rs274_model::rs274_model(boost::program_options::variables_map& vm, const std::string& stock_filename)
 : rs274_base(vm) {
    geom::polyhedron_t model;
    std::ifstream is(stock_filename);
    throw_if(!(is >> geom::format::off >> model), "Unable to read stock from file");
    _model = to_mesh(model);

    auto type = machine_config::get_machine_type(config, machine_id);
    _lathe = type == machine_config::machine_type::lathe;
}

CorkMesh merge(std::vector<CorkMesh> meshes) {
    auto it = begin(meshes);
    CorkMesh res = *it++;
    while (it != end(meshes))
        res.boolUnion(*it++);
    return res;
}

std::vector<CorkMesh> parallel_fold_toolpath(unsigned int n, std::vector<CorkMesh> tool_motion) {
    if(tool_motion.size() == 1) return tool_motion;
    if(n == 1) return { merge(tool_motion) };

    unsigned int chunk_size = std::floor(tool_motion.size() / static_cast<double>(n));
    unsigned int rem = tool_motion.size() % n;
    
    typedef std::future<CorkMesh> polyhedron_future;
    std::vector<polyhedron_future> folded;

    for(unsigned int i = 0; i < n; ++i) {

        std::packaged_task<CorkMesh()> fold([&tool_motion, chunk_size, rem, i]() {
            auto begin = (chunk_size * i) + (i < rem ? i : rem);
            auto end = (begin + chunk_size) + (i < rem ? 1 : 0);

            return merge(std::vector<CorkMesh>(std::make_move_iterator(tool_motion.begin() + begin), std::make_move_iterator(tool_motion.begin() + end)));
        });

        folded.push_back(fold.get_future());
        std::thread(std::move(fold)).detach();
    }

    std::vector<CorkMesh> result;
    std::transform(begin(folded), end(folded), std::back_inserter(result), [](polyhedron_future& f){ return f.get(); });
    return result;
}
CorkMesh parallel_fold_toolpath(std::vector<CorkMesh> tool_motion) {
    auto cores = hardware_concurrency();
    while(cores > 1) {
        tool_motion = parallel_fold_toolpath(cores, tool_motion);
        cores /= 2;
    }
    return merge(tool_motion);
}

static void corkMesh2CorkTriMesh(
    CorkMesh *mesh_in,
    CorkTriMesh *out
) {
    RawCorkMesh raw = mesh_in->raw();
    
    out->n_triangles = raw.triangles.size();
    out->n_vertices  = raw.vertices.size();
    
    out->triangles = new uint[(out->n_triangles) * 3];
    out->vertices  = new float[(out->n_vertices) * 3];
    
    for(uint i=0; i<out->n_triangles; i++) {
        (out->triangles)[3*i+0] = raw.triangles[i].a;
        (out->triangles)[3*i+1] = raw.triangles[i].b;
        (out->triangles)[3*i+2] = raw.triangles[i].c;
    }
    
    for(uint i=0; i<out->n_vertices; i++) {
        (out->vertices)[3*i+0] = raw.vertices[i].pos.x;
        (out->vertices)[3*i+1] = raw.vertices[i].pos.y;
        (out->vertices)[3*i+2] = raw.vertices[i].pos.z;
    }
}

CorkTriMesh rs274_model::model() {
    if(!_toolpath.empty()) {
        auto toolpath = parallel_fold_toolpath(_toolpath);
        //if (_lathe)
        //    std::cerr << geom::format::off << toolpath;
        _toolpath.clear();
        _model.boolDiff(toolpath);
    }
    CorkTriMesh ret;
    corkMesh2CorkTriMesh(&_model, &ret);
    return ret;
}
