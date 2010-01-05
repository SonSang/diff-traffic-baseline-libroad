#include "arc_road.hpp"

mat3x3f axis_angle_matrix(const float theta, const vec3f &axis)
{
    const float c = std::cos(theta);
    const float s = std::sin(theta);
    mat3x3f res;
    res = axis[0]*axis[0] + (1.0-axis[0]*axis[0])*c, axis[0]*axis[1]*(1.0-c) -  axis[2]*s,  axis[0]*axis[2]*(1.0-c) +  axis[1]*s,
          axis[0]*axis[1]*(1.0-c) +  axis[2]*s, axis[1]*axis[1] + (1.0-axis[1]*axis[1])*c,  axis[1]*axis[2]*(1.0-c) -  axis[0]*s,
          axis[0]*axis[2]*(1.0-c) -  axis[1]*s, axis[1]*axis[2]*(1.0-c) +  axis[0]*s,  axis[2]*axis[2] + (1.0-axis[2]*axis[2])*c;
    return res;
}

float cot_theta(const vec3f &nb, const vec3f &nf)
{
    float d = tvmet::dot(nb, nf);
    return std::sqrt( (1.0 + d) / (1.0 - d) );
}

void circle_frame(vec3f &pos, vec3f &tan, const float theta, const mat4x4f &matrix, const float radius)
{
    const float c = std::cos(theta);
    const float s = std::sin(theta);
    const vec4f v(matrix*vec4f(radius*c, radius*s, 0.0, 1.0));
    pos[0] = v[0];
    pos[1] = v[1];
    pos[2] = v[2];
    const vec4f t(matrix * vec4f(-s, c, 0.0, 0.0));
    tan[0] = t[0];
    tan[1] = t[1];
    tan[2] = t[2];
}

arc_road::arc_road(const polyline_road &p)
{
    const size_t N = p.points_.size()-2;

    p_start_   = p.points_.front();
    tan_start_ = p.normals_.front();
    p_end_     = p.points_.back();
    tan_end_   = p.normals_.back();

    frames_.resize(N);
    radii_ .resize(N);
    arcs_  .resize(N);

    for(size_t i = 0; i < N; ++i)
    {
        float alpha = std::min( (p.clengths_[i+1] - p.clengths_[i]), (p.clengths_[i+2] - p.clengths_[i+1]) )/2;
        radii_[i] = alpha * cot_theta(p.normals_[i], p.normals_[i+1]);

        const vec3f laxis(tvmet::normalize(tvmet::cross(p.normals_[i+1], p.normals_[i])));

        frames_[i](0, 2) = laxis[0];
        frames_[i](1, 2) = laxis[1];
        frames_[i](2, 2) = laxis[2];
        frames_[i](3, 2) = 0.0f;

        const mat3x3f rot_pi2(axis_angle_matrix(M_PI_2, laxis));

        const vec3f rm(rot_pi2*-p.normals_[i]);
        const vec3f rp(tvmet::trans(rot_pi2)*p.normals_[i+1]);
        const vec3f up(tvmet::cross(laxis,rm));

        frames_[i](0, 0) = -rm[0];
        frames_[i](1, 0) = -rm[1];
        frames_[i](2, 0) = -rm[2];
        frames_[i](3, 0) = 0.0f;

        frames_[i](0, 1) = up[0];
        frames_[i](1, 1) = up[1];
        frames_[i](2, 1) = up[2];
        frames_[i](3, 1) = 0.0f;

        const vec3f tf(alpha * p.normals_[i+1] + radii_[i]*rp + p.points_[i+1]);
        frames_[i](0, 3) = tf[0];
        frames_[i](1, 3) = tf[1];
        frames_[i](2, 3) = tf[2];
        frames_[i](3, 3) = 1.0f;

        arcs_[i] = M_PI - std::acos(tvmet::dot(-p.normals_[i], p.normals_[i+1]));
    }
}

std::vector<vec3f> arc_road::extract_line(const float offset, const float resolution, const vec3f &up) const
{
    std::vector<vec3f> result;

    const vec3f left0(tvmet::normalize(tvmet::cross(up, tan_start_)));
    result.push_back(vec3f(p_start_ + offset*left0));

    const size_t N = frames_.size();
    for(size_t i = 0; i < N; ++i)
    {
        std::list<std::pair<float,vec3f> > new_points;
        bool new_start;
        {
            vec3f pos;
            vec3f tan;

            circle_frame(pos, tan, 0, frames_[i], radii_[i]);

            const vec3f left(tvmet::normalize(tvmet::cross(up, tan)));
            const vec3f point0(pos + left*offset);

            const vec3f diff(point0 - result.back());
            const float distance(std::sqrt(tvmet::dot(diff, diff)));
            if(distance >= 1e-3)
            {
                new_points.push_back(std::make_pair(0, point0));
                new_start = true;
            }
            else
            {
                new_points.push_back(std::make_pair(0, result.back()));
                new_start = false;
            }
        }
        {
            vec3f pos;
            vec3f tan;

            circle_frame(pos, tan, arcs_[i], frames_[i], radii_[i]);
            const vec3f left(tvmet::normalize(tvmet::cross(up, tan)));
            const vec3f pointend(pos + left*offset);

            new_points.push_back(std::make_pair(arcs_[i], pointend));
        }

        std::list<std::pair<float, vec3f> >::iterator check = new_points.begin();

        while(boost::next(check) != new_points.end())
        {
            const vec3f diff(boost::next(check)->second - check->second);
            const float distance(std::sqrt(tvmet::dot(diff, diff)));
            if (distance > resolution)
            {
                const float new_theta = (boost::next(check)->first + check->first)/2;
                vec3f pos;
                vec3f tan;
                circle_frame(pos, tan, new_theta, frames_[i], radii_[i]);

                const vec3f left(tvmet::normalize(tvmet::cross(up, tan)));
                new_points.insert(boost::next(check), std::make_pair(new_theta, pos + left*offset));
            }
            else
                ++check;
        }

        if(!new_start)
            new_points.pop_front();

        while(!new_points.empty())
        {
            result.push_back(new_points.front().second);
            new_points.pop_front();
        }

    }

    const vec3f leftend(tvmet::normalize(tvmet::cross(up, tan_end_)));
    const vec3f pointend(p_end_ + offset*leftend);
    const vec3f diff(pointend - result.back());
    const float distance(std::sqrt(tvmet::dot(diff, diff)));
    if(distance > 1e-3)
        result.push_back(pointend);

    return result;
}
