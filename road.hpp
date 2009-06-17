#ifndef _ROAD_HPP_
#define _ROAD_HPP_

#include <tvmet/Matrix.h>
#include <tvmet/Vector.h>

typedef tvmet::Vector<float, 3> vec3f;
typedef tvmet::Matrix<float, 3, 3> mat3x3f;
typedef tvmet::Matrix<float, 4, 4> mat4x4f;

struct road_rep
{
    virtual float length     ()                       const = 0;
    virtual void  point      (float t, vec3f   &pt)   const = 0;
    virtual void  frame      (float t, mat3x3f &fr)   const = 0;
    virtual void  point_frame(float t, mat4x4f &ptfr) const = 0;

    virtual ~road_rep() {};

    // some way to draw
    // some way to serialize
};
#endif
