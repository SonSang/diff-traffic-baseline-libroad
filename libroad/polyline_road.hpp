#ifndef _POLYLINE_ROAD_HPP_
#define _POLYLINE_ROAD_HPP_

#include "road_rep.hpp"
#include <vector>

struct polyline_road
{
    float length     ()                       const;
    void  point      (float t, vec3f   &pt)   const;
    void  frame      (float t, mat3x3f &fr)   const;
    void  point_frame(float t, mat4x4f &ptfr) const;

    ~polyline_road();

    void   initialize();
    size_t locate(float t) const;
    size_t locate_scale(float t, float &local) const;
    bool   check() const;

    std::vector<vec3f> points_;
    std::vector<vec3f> normals_;
    std::vector<float> clengths_;

    float inv_len_;
};
#endif
