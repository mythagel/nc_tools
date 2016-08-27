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
 * rs274_model.h
 *
 *  Created on: 2015-05-22
 *      Author: nicholas
 */

#ifndef RS274_MODEL_H_
#define RS274_MODEL_H_
#include "base/rs274_base.h"
#include "Position.h"
#include "geom/polyhedron.h"
#include <string>
#include <vector>
#include "cork/cork.h"
#include "cork/mesh/mesh.h"

struct vecTriMesh : CorkTriMesh {
    std::vector<unsigned int> _triangles;
    std::vector<float> _vertices;

    vecTriMesh();
    vecTriMesh(const vecTriMesh& v);
    vecTriMesh& operator=(const vecTriMesh& v);
    void set();
};


struct CorkTriangle;

struct CorkVertex :
    public MinimalVertexData,
    public RemeshVertexData,
    public IsctVertexData,
    public BoolVertexData
{
    void merge(const CorkVertex &v0, const CorkVertex &v1) {
        double                              a0 = 0.5;
        if(v0.manifold && !v1.manifold)     a0 = 0.0;
        if(!v0.manifold && v1.manifold)     a0 = 1.0;
        double a1 = 1.0 - a0;
        
        pos         = a0 * v0.pos       + a1 * v1.pos;
    }
    void interpolate(const CorkVertex &v0, const CorkVertex &v1) {
        double a0 = 0.5;
        double a1 = 0.5;
        pos         = a0 * v0.pos       + a1 * v1.pos;
    }
    
    
    void isct(IsctVertEdgeTriInput<CorkVertex,CorkTriangle> input)
    {
        Vec2d       a_e     = Vec2d(1,1)/2.0;
        Vec3d       a_t     = Vec3d(1,1,1)/3.0;
        a_e /= 2.0;
        a_t /= 2.0;
    }
    void isct(IsctVertTriTriTriInput<CorkVertex,CorkTriangle> input)
    {
        Vec3d       a[3];
        for(uint k=0; k<3; k++) {
            a[k]    = Vec3d(1,1,1)/3.0;
            a[k] /= 3.0;
        }
        for(uint i=0; i<3; i++) {
          for(uint j=0; j<3; j++) {
        }}
    }
    void isctInterpolate(const CorkVertex &v0, const CorkVertex &v1) {
        double a0 = len(v1.pos - pos);
        double a1 = len(v0.pos - pos);
        if(a0 + a1 == 0.0) a0 = a1 = 0.5; // safety
        double sum = a0+a1;
        a0 /= sum;
        a1 /= sum;
    }
};

struct CorkTriangle :
    public MinimalTriangleData,
    public RemeshTriangleData,
    public IsctTriangleData,
    public BoolTriangleData
{
    void merge(const CorkTriangle &, const CorkTriangle &) {}
    static void split(CorkTriangle &, CorkTriangle &,
                      const CorkTriangle &) {}
    void move(const CorkTriangle &) {}
    void subdivide(SubdivideTriInput<CorkVertex,CorkTriangle> input)
    {
        bool_alg_data = input.pt->bool_alg_data;
    }
};

//using RawCorkMesh = RawMesh<CorkVertex, CorkTriangle>;
//using CorkMesh = Mesh<CorkVertex, CorkTriangle>;
typedef RawMesh<CorkVertex, CorkTriangle>   RawCorkMesh;
typedef Mesh<CorkVertex, CorkTriangle>      CorkMesh;



class rs274_model : public rs274_base
{
private:
    CorkMesh _model;
    geom::polyhedron_t _tool;
    std::vector<CorkMesh> _toolpath;
    unsigned _steps_per_revolution = 360;
    bool _lathe = false;

    virtual void _rapid(const Position& pos);
    virtual void _arc(const Position& end, const Position& center, const cxxcam::math::vector_3& plane, int rotation);
    virtual void _linear(const Position& pos);
	virtual void tool_change(int slot);
	virtual void dwell(double seconds);

public:
	rs274_model(boost::program_options::variables_map& vm, const std::string& stock_filename);

    CorkTriMesh model();

	virtual ~rs274_model() = default;
};

#endif /* RS274_MODEL_H_ */
