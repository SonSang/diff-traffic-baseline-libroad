#ifndef _ARC_ROAD_HPP_
#define _ARC_ROAD_HPP_

#include "libroad_common.hpp"
#include "polyline_road.hpp"

struct vertex
{
    vertex(const vec3f &p, const vec3f &n, const vec2f &t)
        : position(p), normal(n), tex_coord(t)
    {}

    vec3f position;
    vec3f normal;
    vec2f tex_coord;
};

void make_mesh(std::vector<vec3u> &faces, const std::vector<vertex> &vrts,
               const vec2i &low_range, const vec2i &high_range);

struct arc_road
{
    float   length     (float offset) const;
    float   length     (float t, float offset) const;
    vec3f   point      (float t, float offset, const vec3f &up=vec3f(0, 0, 1)) const;
    mat3x3f frame      (float t, float offset, bool reverse, const vec3f &up=vec3f(0, 0, 1)) const;
    mat4x4f point_frame(float t, float offset, bool reverse, const vec3f &up=vec3f(0, 0, 1)) const;
    void    translate  (const vec3f &o);
    void    bounding_box(vec3f &low, vec3f &high) const;

    float parameter_map(float t, float offset) const;
    float length_at_feature(size_t i, float p, float offset) const;

    void extract_arc   (std::vector<vertex> &result, const size_t i, const vec2f &in_range, const float offset, const float resolution, const vec3f &up) const;

    void extract_line  (std::vector<vertex> &result, const vec2f &range, const float offset, const float resolution, const vec3f &up=vec3f(0, 0, 1)) const;
    void extract_center(std::vector<vertex> &result, const vec2f &range, const float offset, const float resolution, const vec3f &up=vec3f(0, 0, 1)) const;
    void make_mesh(std::vector<vertex> &vrts, std::vector<vec3u> &faces,
                   const vec2f &range,
                   const vec2f &offsets, const float resolution) const;

    bool   initialize(const float cull_prox);

    // arc roads are made up of 'features', i.e. alternating straight segments and arcs
    // so even features are segments (freqeuently degenerate) and odd features are arcs
    // there are 2*N+1 features in a arc_road, where n is the number of interor points (i.e. frames_.size())
    // this function returns the length of the road up to the start of the i-th feature
    float  feature_base(size_t i, float offset) const;
    // this function returns the length of the i-th feature itself
    float  feature_size(size_t i, float offset) const;
    size_t locate(float t, float offset) const;
    size_t locate_scale(float t, float offset, float &local) const;

    void   xml_read_as_poly (xmlpp::TextReader &reader, const vec3f &scale=vec3f(1.0f, 1.0f, 1.0f));
    void   xml_write_as_poly(xmlpp::Element *elt) const;
    bool   check() const;

    std::vector<mat4x4f> frames_;
    std::vector<float>   radii_;
    std::vector<float>   arcs_;

    std::vector<float>   seg_clengths_;
    std::vector<vec2f>   arc_clengths_;

    std::vector<vec3f>   points_;
    std::vector<vec3f>   normals_;
};

#endif
