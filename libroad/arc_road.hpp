#ifndef _ARC_ROAD_HPP_
#define _ARC_ROAD_HPP_

#include "libroad_common.hpp"
#include "polyline_road.hpp"

struct arc_road
{
    arc_road(const polyline_road &p);

    float   length     (float offset) const;
    vec3f   point      (float t, float offset) const;
    mat3x3f frame      (float t, float offset) const;
    mat4x4f point_frame(float t, float offset) const;
    void    translate  (const vec3f &o);

    std::vector<vec3f> extract_line(const float offset, const float resolution, const vec3f &up=vec3f(0, 0, 1)) const;
    void make_mesh(std::vector<vec3f> &vrts, std::vector<vec3i> &faces,
                   const float low_offset, const float high_offset, const float resolution) const;

    bool   initialize();
    size_t locate(bool &arcp, float t, float offset) const;
    size_t locate_scale(float t, float &local) const;
    bool   check() const;

    vec3f                p_start_;
    vec3f                tan_start_;
    vec3f                p_end_;
    vec3f                tan_end_;
    std::vector<mat4x4f> frames_;
    std::vector<float>   radii_;
    std::vector<float>   arcs_;
    std::vector<float>   clengths_;
};

#endif
