#include "arc_road.hpp"
#include <boost/bimap.hpp>

static mat3x3f axis_angle_matrix(const float theta, const vec3f &axis)
{
    const float c = std::cos(theta);
    const float s = std::sin(theta);
    mat3x3f res;
    res = axis[0]*axis[0] + (1.0-axis[0]*axis[0])*c, axis[0]*axis[1]*(1.0-c) -  axis[2]*s,  axis[0]*axis[2]*(1.0-c) +  axis[1]*s,
          axis[0]*axis[1]*(1.0-c) +  axis[2]*s, axis[1]*axis[1] + (1.0-axis[1]*axis[1])*c,  axis[1]*axis[2]*(1.0-c) -  axis[0]*s,
          axis[0]*axis[2]*(1.0-c) -  axis[1]*s, axis[1]*axis[2]*(1.0-c) +  axis[0]*s,  axis[2]*axis[2] + (1.0-axis[2]*axis[2])*c;
    return res;
}

static float cot_theta(const vec3f &nb, const vec3f &nf)
{
    float d = tvmet::dot(nb, nf);
    return std::sqrt( (1.0 + d) / (1.0 - d) );
}

static void circle_frame(vec3f &pos, vec3f &tan, const float theta, const mat4x4f &matrix, const float radius)
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

static vec3f triangle_angles(const vec3f &pt0, const vec3f &pt1, const vec3f &pt2)
{
    const vec3f v01(tvmet::normalize(pt1 - pt0));
    const vec3f v02(tvmet::normalize(pt2 - pt0));
    const vec3f v12(tvmet::normalize(pt2 - pt1));

    const float a0 = std::acos(tvmet::dot(v01, v02));
    const float a1 = std::acos(tvmet::dot(-v01, v12));
    const float a2 = 2*M_PI - (a0 + a1);

    return vec3f(a0, a1, a2);
}

static void make_mesh(std::vector<vec3f> &vrts, std::vector<vec3i> &faces,
                      const std::vector<vec3f> &low, const std::vector<vec3f> &high)
{
    vrts.clear();
    faces.clear();

    std::vector<vec3f>::const_iterator low_itr  = low.begin();
    std::vector<vec3f>::const_iterator high_itr = high.begin();

    vrts.push_back(*low_itr++);
    vrts.push_back(*high_itr++);

    size_t base_low_idx  = vrts.size()-2;
    size_t base_high_idx = vrts.size()-1;

    const vec3f *low_cand_vrt  = (low_itr == low.end()   ? 0 : &(*low_itr++));
    const vec3f *high_cand_vrt = (high_itr == high.end() ? 0 : &(*high_itr++));

    bool pick_vrt;
    while(1)
    {
        if(!low_cand_vrt)
        {
            if(!high_cand_vrt)
                break;
            else
                pick_vrt = true;
        }
        else if(!high_cand_vrt)
            pick_vrt = false;
        else
        {
            const vec3f angles_low (triangle_angles(vrts[base_low_idx], vrts[base_high_idx], *low_cand_vrt));
            const vec3f angles_high(triangle_angles(vrts[base_low_idx], vrts[base_high_idx], *high_cand_vrt));
            if(*std::max_element(angles_low.begin(), angles_low.end()) > *std::max_element(angles_high.begin(), angles_high.end()))
                pick_vrt = false;
            else
                pick_vrt = true;
        }

        faces.push_back(vec3i(base_low_idx, base_high_idx, vrts.size()));

        if(pick_vrt)
        {
            vrts.push_back(*high_cand_vrt);
            base_high_idx = vrts.size()-1;
            high_cand_vrt = (high_itr == high.end() ? 0 : &(*high_itr++));
        }
        else
        {
            vrts.push_back(*low_cand_vrt);
            base_low_idx = vrts.size()-1;
            low_cand_vrt = (low_itr == low.end() ? 0 : &(*low_itr++));
        }
    }
}

struct idx_sort
{
    idx_sort(const std::vector<float> &v) : vec(v)
    {
    }

    bool operator()(size_t x, size_t y) const
    {
        return vec[x] < vec[y];
    }

    const std::vector<float> &vec;
};

struct slack_idx_cmp
{
    slack_idx_cmp(const std::vector<float> &v) : vec(v)
    {
    }

    bool operator()(size_t x, size_t y) const
    {
        return vec[x] < vec[y];
    }

    const std::vector<float> &vec;
};

arc_road::arc_road(const polyline_road &p)
{
    const size_t N = p.points_.size()-2;

    std::vector<float> poly_lengths(N+1);
    for(size_t i = 0; i < N+1; ++i)
        poly_lengths[i] = (p.clengths_[i+1] - p.clengths_[i])/2;
    poly_lengths.front() *= 2;
    poly_lengths.back()  *= 2;

    std::vector<size_t> indexes(N+1);
    for(size_t i = 0; i < N+1; ++i)
        indexes[i] = i;
    idx_sort isort(poly_lengths);
    std::sort(indexes.begin(), indexes.end(), isort);

    std::vector<float> alphas(N, 0);
    std::vector<bool>  set(N, false);

    BOOST_FOREACH(size_t idx, indexes)
    {
        if(idx != 0 && !set[idx-1])
        {
            alphas[idx-1] = poly_lengths[idx];
            set[idx-1]    = true;
        }
        if(idx != N && !set[idx])
        {
            alphas[idx] = poly_lengths[idx];
            set[idx]    = true;
        }
    }

    for(size_t i = 0; i < N+1; ++i)
        poly_lengths[i] = (p.clengths_[i+1] - p.clengths_[i]);

    indexes.resize(N);
    for(size_t i = 0; i < N; ++i)
        indexes[i] = i;

    std::vector<float> slacks(N, 0);
    slacks[0] = std::min(poly_lengths[0] - alphas[0],
                         poly_lengths[1] - alphas[1] - alphas[0]);

    for(size_t i = 1; i < N-1; ++i)
    {
        slacks[i] = std::min(poly_lengths[i]   - alphas[i-1] - alphas[i],
                             poly_lengths[i+1] - alphas[i+1] - alphas[i]);
    }

    slacks[N-1] = std::min(poly_lengths[N-1] - alphas[N-2]  - alphas[N-1],
                           poly_lengths[N]                  - alphas[N-1]);
    idx_sort ssort(slacks);

    std::vector<size_t>::iterator current = indexes.begin();

    while(current != indexes.end())
    {
        std::sort(current, indexes.end(), ssort);

        alphas[*current] += slacks[*current];

        if(*current > 0)
        {
            const size_t prev_idx = *current - 1;

            if(prev_idx > 0)
                slacks[prev_idx] = std::min(poly_lengths[prev_idx]   - alphas[prev_idx-1] - alphas[prev_idx],
                                            poly_lengths[prev_idx+1] - alphas[prev_idx+1] - alphas[prev_idx]);
            else
                slacks[prev_idx] = std::min(poly_lengths[prev_idx]   - alphas[prev_idx],
                                            poly_lengths[prev_idx+1] - alphas[prev_idx+1] - alphas[prev_idx]);

        }
        if(*current < N-1)
        {
            const size_t next_idx = *current + 1;

            if(next_idx < N - 1)
                slacks[next_idx] = std::min(poly_lengths[next_idx]   - alphas[next_idx-1] - alphas[next_idx],
                                            poly_lengths[next_idx+1] - alphas[next_idx+1] - alphas[next_idx]);
            else
                slacks[next_idx] = std::min(poly_lengths[next_idx]   - alphas[next_idx-1] - alphas[next_idx],
                                            poly_lengths[next_idx+1] - alphas[next_idx]);
        }
        ++current;
    }

    p_start_   = p.points_.front();
    tan_start_ = p.normals_.front();
    p_end_     = p.points_.back();
    tan_end_   = p.normals_.back();

    frames_.resize(N);
    radii_ .resize(N);
    arcs_  .resize(N);

    clengths_.resize(2*(N+1)+2);

    clengths_[0] = 0.0f;
    clengths_[1] = 0.0f;

    float last_alpha = 0.0f;
    for(size_t i = 0; i < N; ++i)
    {
        float alpha = alphas[i];
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

        clengths_[2*(i+1)]   = clengths_[2*i] + p.clengths_[i+1] - p.clengths_[i] - last_alpha - alpha + radii_[i]*arcs_[i];
        clengths_[2*(i+1)+1] = clengths_[2*i+1] + arcs_[i]*copysign(1.0, laxis[2]);

        last_alpha = alpha;
    }

    clengths_[2*(N+1)]   = clengths_[2*N] + p.clengths_[N+1] - p.clengths_[N] - last_alpha;
    clengths_[2*(N+1)+1] = clengths_[2*N+1];
}

float arc_road::length(const float offset) const
{
    const size_t N = frames_.size();
    return clengths_[2*(N+1)] + offset*clengths_[2*(N+1)+1];
}

vec3f arc_road::point(const float t, const float offset, const vec3f &up) const
{
    vec3f pos;
    vec3f tan;

    float local;
    const size_t idx = locate_scale(t, offset, local);
    if(idx & 1)
    {
        circle_frame(pos, tan, local*arcs_[idx], frames_[idx], radii_[idx]);
        const vec3f left(tvmet::normalize(tvmet::cross(up, tan)));
        return vec3f(pos + left*offset);
    }
    else
    {
        circle_frame(pos, tan, arcs_[idx-1], frames_[idx-1], radii_[idx-1]);
        const vec3f left(tvmet::normalize(tvmet::cross(up, tan)));
        return vec3f(pos + left*offset + tan*local*(clengths_[idx] - offset*clengths_[idx-1]));
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
        std::vector<std::pair<float,vec3f> > new_points;
        {
            vec3f pos;
            vec3f tan;

            circle_frame(pos, tan, arcs_[i], frames_[i], radii_[i]);
            const vec3f left(tvmet::normalize(tvmet::cross(up, tan)));
            const vec3f pointend(pos + left*offset);

            new_points.push_back(std::make_pair(arcs_[i], pointend));
        }
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
                result.push_back(point0);
            }
            else
                new_points.push_back(std::make_pair(0, result.back()));
        }

        while(new_points.size() > 1)
        {
            const std::pair<float, vec3f> &front = new_points[new_points.size()-1];
            const std::pair<float, vec3f> &next  = new_points[new_points.size()-2];

            const vec3f diff(front.second - next.second);
            const float distance(std::sqrt(tvmet::dot(diff, diff)));
            if (distance > resolution)
            {
                const float new_theta = (next.first + front.first)/2;
                vec3f pos;
                vec3f tan;
                circle_frame(pos, tan, new_theta, frames_[i], radii_[i]);

                const vec3f left(tvmet::normalize(tvmet::cross(up, tan)));
                new_points.push_back(std::make_pair(new_theta, pos + left*offset));
                std::swap(new_points[new_points.size()-1], new_points[new_points.size()-2]);
            }
            else
            {
                new_points.pop_back();
                result.push_back(new_points.back().second);
            }
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

void arc_road::make_mesh(std::vector<vec3f> &vrts, std::vector<vec3i> &faces,
                         const float low_offset, const float high_offset, const float resolution) const
{
    ::make_mesh(vrts, faces, extract_line(low_offset, resolution), extract_line(high_offset, resolution));
}

size_t arc_road::locate(const float t, const float offset) const
{
    const float scaled_t = t*length(offset);

    size_t low = 0;
    size_t high = clengths_.size();
    while (low < high)
    {
        const size_t mid = low + ((high - low) / 2);
        float lookup;
        if (mid & 1) // mid is odd
            lookup = clengths_[mid-1] + offset*clengths_[mid];
        else if(mid)
            lookup = offset*clengths_[mid-1] + clengths_[mid];
        else
            lookup = 0.0f;

        if (lookup < scaled_t)
            low = mid + 1;
        else
            high = mid;
    }
    return low;
}

size_t arc_road::locate_scale(const float t, const float offset, float &local) const
{
    const float scaled_t = t*length(offset);

    size_t low  = 0;
    size_t high = clengths_.size();
    while (low < high)
    {
        const size_t mid = low + ((high - low) / 2);
        float lookup;
        if (mid & 1) // mid is odd
            lookup = clengths_[mid-1] + offset*clengths_[mid];
        else if(mid)
            lookup = offset*clengths_[mid-1] + clengths_[mid];
        else
            lookup = 0.0f;

        if (lookup < scaled_t)
            low = mid + 1;
        else
            high = mid;
    }

    float lookup;
    float base;
    if(low & 1) // low is odd
    {
        lookup = clengths_[low-1] + offset*clengths_[low];
        base   = clengths_[low+1];
    }
    else if(low)
    {
        lookup = offset*clengths_[low-1] + clengths_[low];
        base   = offset*clengths_[low+1];
    }
    else
    {
        lookup = 0.0f;
        base   = clengths_[1];
    }

    local = (scaled_t - lookup) / base;

    return low;
}
