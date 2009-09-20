#include "polyline_road.hpp"

#include <algorithm>
#include <iostream>
#include <cassert>

float polyline_road::length() const
{
    return clengths_.back();
}

void polyline_road::point(float t, vec3f &pt) const
{
    float local;
    size_t idx = locate_scale(t, local);
    pt = points_[idx] + local*normals_[idx];
}

void polyline_road::frame(float t, mat3x3f &fr) const
{
    float local;
    size_t idx = locate_scale(t, local);

    //* Equal to tvmet::cross(normals_[idX], vec3f(0,0,1));
    vec3f right(normals_[idx][1], -normals_[idx][0], 0.0f);
    float rightlen = std::sqrt(tvmet::dot(right, right));
    right /= rightlen;

    vec3f up(tvmet::cross(right, normals_[idx]));
    assert(std::abs(tvmet::dot(up, up) - 1.0f) < 1e-7f);

    fr =
        normals_[idx][0], right[0], up[0],
        normals_[idx][1], right[1], up[1],
        normals_[idx][2], right[2], up[2];
}

void polyline_road::point_frame(float t, mat4x4f &fr) const
{
    float local;
    size_t idx = locate_scale(t, local);

    //* Equal to tvmet::cross(normals_[idX], vec3f(0,0,1));
    vec3f right(normals_[idx][1], -normals_[idx][0], 0.0f);
    float rightlen = std::sqrt(tvmet::dot(right, right));
    right /= rightlen;

    vec3f up(tvmet::cross(right, normals_[idx]));
    assert(std::abs(tvmet::dot(up, up) - 1.0f) < 1e-7);

    fr =
        normals_[idx][0], right[0], up[0], points_[idx][0] + local*normals_[idx][0],
        normals_[idx][1], right[1], up[1], points_[idx][1] + local*normals_[idx][1],
        normals_[idx][2], right[2], up[2], points_[idx][2] + local*normals_[idx][2],
        0.0f,             0.0f,     0.0f,  1.0f;
}

polyline_road::~polyline_road()
{
}

void polyline_road::initialize()
{
    clengths_.resize(points_.size());
    normals_.resize(points_.size()-1);

    clengths_[0] = 0.0f;
    for(size_t i = 1; i < points_.size(); ++i)
    {
        normals_[i-1] = points_[i] - points_[i-1];
        float len = tvmet::dot(normals_[i-1], normals_[i-1]);
        if(len > 0.0f)
            len = std::sqrt(len);

        clengths_[i] = clengths_[i-1] + len;

        normals_[i-1] /= len;
    }

    inv_len_ = 1.0f/clengths_.back();
}

inline size_t polyline_road::locate(float t) const
{
    float scale_t = t*length();
    std::vector<float>::const_iterator found = std::upper_bound(clengths_.begin(), clengths_.end(), scale_t);
    size_t idx = found - clengths_.begin();
    if(idx > 0)
        return idx - 1;
    else
        return 0;
}

inline size_t polyline_road::locate_scale(float t, float &local) const
{
    float scale_t = t*length();
    std::vector<float>::const_iterator found = std::upper_bound(clengths_.begin(), clengths_.end(), scale_t);
    size_t idx = found - clengths_.begin();
    if(idx > 0)
    {
        local = (scale_t - clengths_[idx-1]) / (clengths_[idx] - clengths_[idx-1]);
        return idx - 1;
    }
    else
    {
        local = 0.0f;
        return 0;
    }
}
