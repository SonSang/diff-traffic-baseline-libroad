#include "polyline_road.hpp"
#include "svg_helper.hpp"

#include <iostream>

polyline_road::~polyline_road()
{
}

float polyline_road::length(const float offset) const
{
    return clengths_.back() - 2*offset*cmitres_.back();
}

vec3f polyline_road::point(const float t, const float offset) const
{
    const size_t idx = locate(t, offset);

    const float last_cmitre = idx == 0 ? 0 : cmitres_[idx-1];
    const float mitre       = cmitres_[idx] - last_cmitre;

    const float local_t = t*length(offset) - (clengths_[idx] - offset*(cmitres_[idx] + last_cmitre));

    const vec3f right(normals_[idx][1], -normals_[idx][0], 0.0f);
    return vec3f(points_[idx] + (local_t + offset*mitre)*normals_[idx] - offset*right);
}

mat3x3f polyline_road::frame(const float t, const float offset) const
{
    const size_t idx = locate(t, offset);

    //* Equal to tvmet::cross(normals_[idX], vec3f(0,0,1));
    const vec3f right(normals_[idx][1], -normals_[idx][0], 0.0f);
    const vec3f up(tvmet::cross(right, normals_[idx]));

    mat3x3f res;
    res = normals_[idx][0], right[0], up[0],
          normals_[idx][1], right[1], up[1],
          normals_[idx][2], right[2], up[2];

    return res;
}

mat4x4f polyline_road::point_frame(const float t, const float offset) const
{
    const size_t idx = locate(t, offset);

    //* Equal to tvmet::cross(normals_[idX], vec3f(0,0,1));
    const vec3f right(normals_[idx][1], -normals_[idx][0], 0.0f);
    const vec3f up(tvmet::cross(right, normals_[idx]));

    const float last_cmitre = idx == 0 ? 0 : cmitres_[idx-1];
    const float mitre       = cmitres_[idx] - last_cmitre;

    const float local_t = t*length(offset) - (clengths_[idx] - offset*(cmitres_[idx] + last_cmitre));

    const vec3f point(points_[idx] + (local_t + offset*mitre)*normals_[idx] - offset*right);

    mat4x4f fr;
    fr = normals_[idx][0], right[0], up[0], point[0],
         normals_[idx][1], right[1], up[1], point[1],
         normals_[idx][2], right[2], up[2], point[2],
         0.0f,             0.0f,     0.0f,  1.0f;
    return fr;
}

void polyline_road::translate(const vec3f &o)
{
    BOOST_FOREACH(vec3f &pt, points_)
    {
        pt += o;
    }
}

bool polyline_road::initialize()
{
    const size_t N = points_.size();
    clengths_.resize(N);
    normals_.resize(N-1);

    clengths_[0] = 0.0f;
    for(size_t i = 1; i < N; ++i)
    {
        normals_[i-1] = points_[i] - points_[i-1];
        const float len = ::length(normals_[i-1]);
        if(len < FLT_EPSILON)
            return false;

        clengths_[i] = clengths_[i-1] + len;
        normals_[i-1] /= len;
    }
    inv_len_ = 1.0f/clengths_.back();

    cmitres_.resize(N);

    cmitres_[0] = 0.0f;
    if (N >= 2){
        for(size_t i = 0; i < N - 2; ++i)
        {
            const float dot = tvmet::dot(normals_[i], normals_[i+1]);
            if(std::abs(dot + 1.0f) < FLT_EPSILON)
                return false;
            const float orient = normals_[i][1] * normals_[i+1][0] - normals_[i][0]*normals_[i+1][1];
            const float mitre  = (dot > 1.0f) ? 0.0f : copysign(std::sqrt((1.0f - dot)/(1.0f + dot)), -orient);

            cmitres_[i+1] += cmitres_[i] + mitre;
        }
    }


    cmitres_[N-1] = cmitres_[N-2]; // + 0.0f;

    return true;
}

inline size_t polyline_road::locate(const float t, const float offset) const
{
    const float                              scale_t = t*length(offset);
    const std::vector<float>::const_iterator found   = std::upper_bound(clengths_.begin(), clengths_.end(), scale_t);
    const size_t                             idx     = found - clengths_.begin();
    return (idx > 0) ? idx - 1 : 0;
}

inline size_t polyline_road::locate_scale(float &local_t, const float offset, const float t) const
{
    const float                              scale_t = t*length(offset);
    const std::vector<float>::const_iterator found   = std::upper_bound(clengths_.begin(), clengths_.end(), scale_t);
    const size_t                             idx     = found - clengths_.begin();
    if(idx > 0)
    {
        local_t = (scale_t - clengths_[idx-1]) / (clengths_[idx] - clengths_[idx-1]);
        return idx - 1;
    }
    else
    {
        local_t = 0.0f;
        return 0;
    }
}

bool polyline_road::check() const
{
    return (!points_.empty() &&
            points_.size() == clengths_.size() &&
            points_.size() == cmitres_.size() &&
            points_.size() == normals_.size() + 1);
}

str polyline_road::svg_poly_path_center(const vec2f &interval, const float offset, const bool continuation) const
{
    return svg_poly_path(interval, offset, continuation);
}

str polyline_road::svg_poly_path       (const vec2f &interval, const float offset, const bool start) const
{
    vec2f range(interval);
    bool reversed = range[0] > range[1];
    if(reversed)
        std::swap(range[0], range[1]);

    const size_t start_idx = std::min(normals_.size()-1, locate(range[0], offset));
    const size_t end_idx   = std::min(normals_.size()-1, locate(range[1], offset));

    path p;

    vec3f last_point;
    {
        const float last_cmitre = start_idx == 0 ? 0 : cmitres_[start_idx-1];
        const float mitre       = cmitres_[start_idx] - last_cmitre;

        const float local_t = range[0]*length(offset) - (clengths_[start_idx] - offset*(cmitres_[start_idx] + last_cmitre));

        const vec3f right(normals_[start_idx][1], -normals_[start_idx][0], 0.0f);
        last_point = points_[start_idx] + (local_t + offset*mitre)*normals_[start_idx] - offset*right;
    }
    for(int c = start_idx + 1; c <= static_cast<int>(end_idx); ++c)
    {
        const float mitre = cmitres_[c] - cmitres_[c-1];

        const vec3f right(normals_[c][1], -normals_[c][0], 0.0f);
        const vec3f new_point(points_[c] + offset*(mitre*normals_[c] - right));
        p.add_line(last_point, new_point);
        last_point = new_point;
    }
    {
        const float last_cmitre = end_idx == 0 ? 0 : cmitres_[end_idx-1];
        const float mitre       = cmitres_[end_idx] - last_cmitre;

        const float local_t = range[1]*length(offset) - (clengths_[end_idx] - offset*(cmitres_[end_idx] + last_cmitre));

        const vec3f right(normals_[end_idx][1], -normals_[end_idx][0], 0.0f);
        p.add_line(last_point, vec3f(points_[end_idx] + (local_t + offset*mitre)*normals_[end_idx] - offset*right));
    }

    if(reversed)
        p.reverse();

    p.remove_duplicates();
    return p.stringify(start);
}
