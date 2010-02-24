#ifndef _POLYLINE_ROAD_HPP_
#define _POLYLINE_ROAD_HPP_

#include "libroad_common.hpp"

struct polyline_road
{
    ~polyline_road();

    float   length     (float offset)          const;
    vec3f   point      (float t, float offset) const;
    mat3x3f frame      (float t, float offset) const;
    mat4x4f point_frame(float t, float offset) const;
    void    translate(const vec3f &o);

    bool   initialize();
    size_t locate(float t, float offset) const;
    size_t locate_scale(float &local, float t, float offset) const;

    void   xml_read (xmlpp::TextReader &reader, const vec3f &scale=vec3f(1.0f, 1.0f, 1.0f));
    void   xml_write(xmlpp::Element *elt) const;
    bool   check() const;

    std::vector<vec3f> points_;
    std::vector<vec3f> normals_;
    std::vector<float> clengths_;
    std::vector<float> cmitres_;

    str svg_poly_path_center(const vec2f &interval, const float offset, const bool start) const;
    str svg_poly_path       (const vec2f &interval, const float offset, const bool start) const;

    float inv_len_;
};
#endif
