#ifndef CIRCLE3D_H
#define CIRCLE3D_H

namespace circle3d
{

struct Point3D
{
    double x,y,z;
};

void estimate3DCircle(Point3D p1, Point3D p2, Point3D p3, Point3D& c, double& radius);

}

#endif
