#include "polyline_road.hpp"

polyline_road::~polyline_road()
{
}

float polyline_road::length() const
{
    return clengths_.back();
}

vec3f polyline_road::point(const float t) const
{
    float        local_t;
    const size_t idx = locate_scale(t, local_t);
    return vec3f(points_[idx] + local_t*normals_[idx]);
}

mat3x3f polyline_road::frame(const float t) const
{
    float local_t;
    const size_t idx = locate_scale(t, local_t);

    //* Equal to tvmet::cross(normals_[idX], vec3f(0,0,1));
    const float rightlen = 1.0f/std::sqrt(normals_[idx][0]*normals_[idx][0] + normals_[idx][1]*normals_[idx][1]);
    const vec3f right(normals_[idx][1]*rightlen, -normals_[idx][0]*rightlen, 0.0f);
    const vec3f up(tvmet::cross(right, normals_[idx]));

    mat3x3f res;
    res = normals_[idx][0], right[0], up[0],
          normals_[idx][1], right[1], up[1],
          normals_[idx][2], right[2], up[2];

    return res;
}

mat4x4f polyline_road::point_frame(const float t) const
{
    float local_t;
    const size_t idx = locate_scale(t, local_t);

    //* Equal to tvmet::cross(normals_[idX], vec3f(0,0,1));
    const float rightlen = 1.0f/std::sqrt(normals_[idx][0]*normals_[idx][0] + normals_[idx][1]*normals_[idx][1]);
    const vec3f right(normals_[idx][1]*rightlen, -normals_[idx][0]*rightlen, 0.0f);
    const vec3f up(tvmet::cross(right, normals_[idx]));

    mat4x4f fr;
    fr = normals_[idx][0], right[0], up[0], points_[idx][0] + local_t*normals_[idx][0],
         normals_[idx][1], right[1], up[1], points_[idx][1] + local_t*normals_[idx][1],
         normals_[idx][2], right[2], up[2], points_[idx][2] + local_t*normals_[idx][2],
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
        const float len = std::sqrt(tvmet::dot(normals_[i-1], normals_[i-1]));
        if(len < FLT_EPSILON)
            return false;

        clengths_[i] = clengths_[i-1] + len;
        normals_[i-1] /= len;
    }
    inv_len_ = 1.0f/clengths_.back();

    cmitres_.resize(N);

    cmitres_[0] = 0.0f;
    for(size_t i = 0; i < N - 2; ++i)
    {
        const float dot = tvmet::dot(normals_[i], normals_[i+1]);
        if(std::abs(dot + 1.0f) < FLT_EPSILON)
            return false;
        const float orient = normals_[i][1] * normals_[i+1][0] - normals_[i][0]*normals_[i+1][1];
        const float mitre  = (dot > 1.0f) ? 0.0f : copysign(std::sqrt((1.0f - dot)/(1.0f + dot)), orient);

        cmitres_[i+1] += cmitres_[i] + mitre;
    }

    cmitres_[N-1] = cmitres_[N-2]; // + 0.0f;

    return true;
}

inline size_t polyline_road::locate(const float t) const
{
    const float                              scale_t = t*length();
    const std::vector<float>::const_iterator found   = std::upper_bound(clengths_.begin(), clengths_.end(), scale_t);
    const size_t                             idx     = found - clengths_.begin();
    return (idx > 0) ? idx - 1 : 0;
}

inline size_t polyline_road::locate_scale(const float t, float &local_t) const
{
    const float                              scale_t = t*length();
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
            points_.size() == normals_.size() + 1);
}
