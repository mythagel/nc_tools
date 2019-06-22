// MIT License
//
// Copyright (c) 2018 Sergio Garrido-Jurado (sgarrido2011@gmail.com)
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "circle3d.hpp"
#include <math.h>

namespace circle3d
{

/**
 * @brief Basic 3D vector class
 */
class Vector3D
{
public:
    double x,y,z;

    Vector3D(double _x=0, double _y=0, double _z=0)
        : x(_x), y(_y), z(_z)
    {
    }

    double dot(Vector3D v2)
    {
        return x*v2.x + y*v2.y + z*v2.z;
    }

    Vector3D operator+(const Vector3D& v2)
    {
       return Vector3D(x+v2.x, y+v2.y, z+v2.z);
    }

    Vector3D operator-(const Vector3D& v2)
    {
       return Vector3D(x-v2.x, y-v2.y, z-v2.z);
    }

    Vector3D operator*(const double& sc)
    {
       return Vector3D(sc*x, sc*y, sc*z);
    }

    double norm()
    {
        return sqrt(dot(*this));
    }

};


/**
 * @brief estimate the center and the radious of a circle in 3D given 3 points
 * This is the relevant function
 */
static void estimate3DCircle(Vector3D p1, Vector3D p2, Vector3D p3, Vector3D &c, double &radius)
{
    Vector3D v1 = p2-p1;
    Vector3D v2 = p3-p1;
    double v1v1, v2v2, v1v2;
    v1v1 = v1.dot(v1);
    v2v2 = v2.dot(v2);
    v1v2 = v1.dot(v2);

    float base = 0.5/(v1v1*v2v2-v1v2*v1v2);
    float k1 = base*v2v2*(v1v1-v1v2);
    float k2 = base*v1v1*(v2v2-v1v2);
    c = p1 + v1*k1 + v2*k2; // center

    radius = (c-p1).norm();
}

void estimate3DCircle(Point3D p1, Point3D p2, Point3D p3, Point3D& c, double& radius)
{
    Vector3D center;
    estimate3DCircle({p1.x, p1.y, p1.z}, {p2.x, p2.y, p2.z}, {p3.x, p3.y, p3.z}, center, radius);
    c.x = center.x;
    c.y = center.y;
    c.z = center.z;
}

}
