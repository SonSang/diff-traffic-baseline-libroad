#include "arc_road.hpp"
#include "svg_helper.hpp"

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

    const float d0 = std::max(-1.0f, std::min(1.0f, tvmet::dot(v01, v02)));
    const float d1 = std::max(-1.0f, std::min(1.0f, tvmet::dot(-v01, v12)));

    const float a0 = std::acos(d0);
    const float a1 = std::acos(d1);
    const float a2 = M_PI - (a0 + a1);

    assert(std::isfinite(a0));
    assert(std::isfinite(a1));
    assert(std::isfinite(a2));

    return vec3f(a0, a1, a2);
}

static int triangle_cmp(const vec3f &low, const vec3f&high)
{
    // designed for smallest largest angle
    for(int i = 2; i > -1; --i)
    {
        float diff = low[i] - high[i];
        if(std::abs(diff) < 1e-6)
            continue;
        else if(diff < 0.0)
            return 1;
        else
            return -1;
    }
    return 0;
}

void make_mesh(std::vector<vec3u> &faces, const std::vector<vertex> &vrts,
               const vec2i &low_range, const vec2i &high_range)
{
    int low_dir  = copysign(1, low_range[1] -low_range[0]);
    int high_dir = copysign(1, high_range[1]-high_range[0]);

    int low_itr  = low_range[0];
    if(low_dir < 0)
        low_itr += low_dir;

    int high_itr = high_range[0];
    if(high_dir < 0)
        high_itr += high_dir;

    int base_low_idx  = low_itr;
    int base_high_idx = high_itr;

    low_itr  += low_dir;
    high_itr += high_dir;

    const vertex *low_cand_vrt;
    if( (low_dir < 0 && low_itr < low_range[1]) || (low_dir > 0 && low_itr >= low_range[1]) )
        low_cand_vrt = 0;
    else
    {
        low_cand_vrt = &(vrts[low_itr]);
        low_itr += low_dir;
    }

    const vertex *high_cand_vrt;
    if( (high_dir < 0 && high_itr < high_range[1]) || (high_dir > 0 && high_itr >= high_range[1]) )
        high_cand_vrt = 0;
    else
    {
        high_cand_vrt = &(vrts[high_itr]);
        high_itr += high_dir;
    }

    bool last_pick = false;
    bool pick_vrt = false;
    while(1)
    {
        if(!low_cand_vrt)
        {
            if(!high_cand_vrt)
                break;
            else
            {
                last_pick = pick_vrt;
                pick_vrt = true;
            }
        }
        else if(!high_cand_vrt)
        {
            last_pick = pick_vrt;
            pick_vrt = false;
        }
        else
        {
            vec3f angles_low (triangle_angles(vrts[base_low_idx].position, vrts[base_high_idx].position, low_cand_vrt->position));
            vec3f angles_high(triangle_angles(vrts[base_low_idx].position, vrts[base_high_idx].position, high_cand_vrt->position));

            std::sort(angles_low.begin(), angles_low.end());
            std::sort(angles_high.begin(), angles_high.end());

            int res = triangle_cmp(angles_low, angles_high);
            last_pick = pick_vrt;
            if(res > 0)
                pick_vrt = false;
            else if(res < 0)
                pick_vrt = true;
            else
                pick_vrt = !last_pick;
        }

        if(pick_vrt)
        {
            faces.push_back(vec3u(base_low_idx, base_high_idx, high_cand_vrt - &(vrts[0])));
            base_high_idx = high_cand_vrt - &(vrts[0]);
            if( (high_dir < 0 && high_itr < high_range[1]) || (high_dir > 0 && high_itr >= high_range[1]) )
                high_cand_vrt = 0;
            else
            {
                high_cand_vrt = &(vrts[high_itr]);
                high_itr += high_dir;
            }
        }
        else
        {
            faces.push_back(vec3u(base_low_idx, base_high_idx, low_cand_vrt - &(vrts[0])));
            base_low_idx = low_cand_vrt - &(vrts[0]);
            if( (low_dir < 0 && low_itr < low_range[1]) || (low_dir > 0 && low_itr >= low_range[1]) )
                low_cand_vrt = 0;
            else
            {
                low_cand_vrt = &(vrts[low_itr]);
                low_itr += low_dir;
            }
        }
    }
}

static void alpha_assign(std::vector<float> &alphas, const std::vector<float> &seg_lengths, const std::vector<float> &factors, const size_t start, const size_t end)
{
    // We have N_pts points
    // We have N_segs segments (N_pts - 1)
    // We have N_frames interior points (N_pts - 2)
    // start/end are segment indices (so initially 0/N_segs)
    // The i-th segment is bounded by the i-1-th factor/alpha on the low and the i-th factor/alpha on the high

    if(!end || start >= end-1)
        return;

    // assert(start != 0);
    // assert(end != seg_lengths.size());

    float  min_radius = FLT_MAX;
    vec2f  alphas_pick;
    size_t min_seg     = end;

    {
        const size_t c           = start;
        const float  a           = start > 0 ? alphas[c-1] : 0.0f;
        const float  b           = std::min(seg_lengths[c] - a, seg_lengths[c+1]);
        const float  this_radius = factors[c]*b;
        assert(this_radius != 0);
        if(this_radius < min_radius)
        {
            min_radius  = this_radius;
            min_seg     = c;
            alphas_pick = vec2f(a, b);
        }
    }

    for(size_t c = start+1; c < end-1; ++c)
    {
        const float a           = std::min(factors[c] * seg_lengths[c] / (factors[c-1] + factors[c]),
                                           seg_lengths[c-1]);
        const float b           = std::min(seg_lengths[c] - a,
                                           seg_lengths[c+1]);
        const float radius_a    = factors[c-1]*a;
        const float radius_b    = factors[c]*b;
        const float this_radius = std::max(radius_a, radius_b);
        assert(this_radius != 0);
        if(this_radius < min_radius)
        {
            min_radius  = this_radius;
            min_seg     = c;
            alphas_pick = vec2f(a, b);
        }
    }

    {
        const size_t c           = end-1;
        const float  b           = c < seg_lengths.size()-1 ? alphas[c] : 0.0f;
        const float  a           = std::min(seg_lengths[c] - b, seg_lengths[c-1]);
        const float  this_radius = factors[c-1]*a;
        assert(this_radius != 0);
        if(this_radius < min_radius)
        {
            min_radius  = this_radius;
            min_seg     = c;
            alphas_pick = vec2f(a, b);
        }
    }

    assert(min_seg < end);

    if(min_seg > start)
        alphas[min_seg-1] = alphas_pick[0];
    if(min_seg < end-1)
        alphas[min_seg]   = alphas_pick[1];

#ifndef NDEBUG
    for(size_t i = 1; i < seg_lengths.size()-1; ++i)
        assert( seg_lengths[i] - alphas[i-1] - alphas[i] >= -1e5);
#endif

    alpha_assign(alphas, seg_lengths, factors, start, min_seg);
    alpha_assign(alphas, seg_lengths, factors, min_seg+1, end);
}

static std::vector<vec3f> remove_colinear(const std::vector<vec3f> &v, const float eps=1e-6)
{
    if(v.size() < 2)
        return v;

    std::vector<vec3f> res;
    res.push_back(v.front());

    vec3f last_normal(tvmet::normalize(v[1] - v[0]));
    for(size_t i = 1; i < v.size()-1; ++i)
    {
        vec3f normal(tvmet::normalize(v[i+1] - v[i]));
        const vec3f cp(tvmet::cross(last_normal, normal));
        if(tvmet::dot(cp, cp) > eps)
        {
            res.push_back(v[i]);
            last_normal = normal;
        }
        else
            last_normal = vec3f(tvmet::normalize(v[i+1] - res.back()));
    }
    res.push_back(v.back());
    return res;
}

static std::vector<vec3f> remove_proximity(const std::vector<vec3f> &v, const float eps2)
{
    if(v.size() < 3)
        return v;

    std::vector<vec3f> res;
    res.push_back(v.front());

    for(size_t i = 1; i < v.size(); ++i)
        if(distance2(v[i], res.back()) > eps2)
            res.push_back(v[i]);

    return res;
}

static vec3f center(const vec3f &point, const vec3f &normal0, const vec3f &normal1, const float radius)
{
    const float alpha(radius/cot_theta(normal0, normal1));
    const vec3f plane(tvmet::normalize(tvmet::cross(normal0, normal1)));
    const vec3f support(tvmet::normalize(tvmet::cross(plane, normal1)));

    return vec3f(point + normal1*alpha + support*radius);
}

static vec3f inv_center(const vec3f &center, const vec3f &normal0, const vec3f &normal1, const float radius)
{
    const float alpha(radius/cot_theta(normal0, normal1));
    const vec3f plane(tvmet::normalize(tvmet::cross(normal0, normal1)));
    const vec3f support(tvmet::normalize(tvmet::cross(plane, normal1)));

    return vec3f(center - normal1*alpha - support*radius);
}

vec3f arc_road::center(const size_t p) const
{
    assert(p > 0 && p < normals_.size());
    return ::center(points_[p], normals_[p-1], normals_[p], radii_[p-1]);
}

bool arc_road::initialize_from_polyline(const float cull_prox, const std::vector<vec3f> &points)
{
    points_ = remove_colinear(points);
    points_ = remove_proximity(points_, cull_prox*cull_prox);

    std::vector<float> lengths, factors;
    if(!compute_geometric(lengths, factors))
        return false;

    std::vector<float> alphas(points_.size()-2, 0);
    alpha_assign(alphas, lengths, factors, 0, lengths.size());

    radii_.resize(alphas.size());
    for(size_t i = 0; i < alphas.size(); ++i)
        radii_[i] = factors[i]*alphas[i];

    remove_redundant();

    if(!compute_geometric(lengths, factors))
        return false;

    alphas.resize(radii_.size());
    for(size_t i = 0; i < alphas.size(); ++i)
        alphas[i] = radii_[i]/factors[i];

    return initialize(alphas, lengths);
}

bool arc_road::initialize_from_points_radii(const std::vector<vec3f> &points, const std::vector<float> &radii)
{
    if(points.size() != radii_.size() + 2)
        return false;

    points_ = points;
    radii_  = radii;

    std::vector<float> lengths, factors;
    if(!compute_geometric(lengths, factors))
        return false;

    remove_redundant();

    if(!compute_geometric(lengths, factors))
        return false;

    std::vector<float> alphas(points_.size()-2, 0);
    for(size_t i = 0; i < alphas.size(); ++i)
        alphas[i] = radii_[i]/factors[i];

    return initialize(alphas, lengths);
}

bool arc_road::compute_geometric(std::vector<float> &lengths, std::vector<float> &factors)
{
    const size_t N_pts  = points_.size();
    const size_t N_segs = N_pts - 1;
    const size_t N_arcs = N_pts - 2;

    normals_.resize(N_segs);
    lengths.resize(N_segs);
    for(size_t i = 1; i < N_pts; ++i)
    {
        normals_[i-1]    = points_[i] - points_[i-1];
        const float len  = std::sqrt(tvmet::dot(normals_[i-1], normals_[i-1]));
        if(len < FLT_EPSILON)
            return false;
        lengths[i-1]     = len;
        normals_[i-1]   /= len;
    }

    factors.resize(N_arcs);
    for(size_t i = 0; i < N_arcs; ++i)
        factors[i] = cot_theta(normals_[i], normals_[i+1]);

    return true;
}

void arc_road::remove_redundant()
{
    if(points_.size() <= 3)
        return;

    std::vector<vec3f> new_points;
    std::vector<float> new_radii;
    new_points.push_back(points_.front());

    vec3f  last_center(center(1));
    float  last_radius(radii_[0]);
    size_t last_interior = 0;
    for(size_t i = 1; i < radii_.size(); ++i)
    {
        const vec3f new_center(center(i+1));
        if(std::abs(radii_[i] - last_radius) < 1e-4 && distance2(new_center, last_center) < 1e-5 &&
           (M_PI - std::acos(tvmet::dot(-normals_[last_interior], normals_[i+1])) < 7.0/8.0*M_PI))
            continue;

        new_radii.push_back(last_radius);
        if(last_interior+1 != i)
            new_points.push_back(inv_center(last_center, normals_[last_interior], normals_[i], last_radius));
        else
            new_points.push_back(points_[last_interior+1]);

        last_center   = new_center;
        last_radius   = radii_[i];
        last_interior = i;
    }

    new_radii.push_back(last_radius);
    if(last_interior+1 < radii_.size())
        new_points.push_back(inv_center(last_center, normals_[last_interior], normals_.back(), last_radius));
    else
        new_points.push_back(points_[last_interior+1]);

    new_points.push_back(points_.back());

    radii_.swap(new_radii);
    points_.swap(new_points);
}

bool arc_road::initialize(const std::vector<float> &alphas, std::vector<float> &lengths)
{
    const size_t N_pts  = points_.size();
    const size_t N_segs = N_pts - 1;
    const size_t N_arcs = N_pts - 2;

    // Now compute actual helper data
    frames_.resize(N_arcs);
    arcs_  .resize(N_arcs);

    for(size_t i = 0; i < N_arcs; ++i)
    {
        const float alpha  = alphas[i];
        const float radius = radii_[i];
        assert(std::isfinite(radius));

        const vec3f   laxis(tvmet::normalize(tvmet::cross(normals_[i+1], normals_[i])));
        const mat3x3f rot_pi2(axis_angle_matrix(M_PI_2, laxis));
        const vec3f   rm(rot_pi2*-normals_[i]);
        const vec3f   rp(tvmet::trans(rot_pi2)*normals_[i+1]);
        const vec3f   up(tvmet::cross(laxis,rm));
        const vec3f   tf(alpha * normals_[i+1] + radius*rp + points_[i+1]);

        frames_[i](0, 0) = -rm[0]; frames_[i](0, 1) = up[0]; frames_[i](0, 2) = laxis[0]; frames_[i](0, 3) = tf[0];
        frames_[i](1, 0) = -rm[1]; frames_[i](1, 1) = up[1]; frames_[i](1, 2) = laxis[1]; frames_[i](1, 3) = tf[1];
        frames_[i](2, 0) = -rm[2]; frames_[i](2, 1) = up[2]; frames_[i](2, 2) = laxis[2]; frames_[i](2, 3) = tf[2];
        frames_[i](3, 0) =   0.0f; frames_[i](3, 1) =  0.0f; frames_[i](3, 2) = 0.0f;     frames_[i](3, 3) =  1.0f;

        arcs_[i] = M_PI - std::acos(tvmet::dot(-normals_[i], normals_[i+1]));
    }

    arc_clengths_.resize(N_segs);
    arc_clengths_[0] = vec2f(0.0f, 0.0f);
    for(size_t i = 0; i < N_arcs; ++i)
        arc_clengths_[i+1] = arc_clengths_[i] + vec2f(radii_[i]*arcs_[i], arcs_[i]*copysign(1.0, frames_[i](2, 2)));

    for(size_t i = 0; i < N_arcs; ++i)
    {
        lengths[i]   -= alphas[i];
        lengths[i+1] -= alphas[i];
    }

    seg_clengths_.resize(N_pts);
    seg_clengths_[0] = 0.0f;
    for(size_t i = 1; i < N_pts; ++i)
        seg_clengths_[i] = seg_clengths_[i-1] + lengths[i-1];

    return true;
}

float arc_road::length(const float offset) const
{
    return feature_base(2*frames_.size()+1, offset);
}

float arc_road::length(const float t, const float offset) const
{
    float center_local;
    size_t feature_idx = locate_scale(t, 0.0f, center_local);

    return length_at_feature(feature_idx, center_local, offset);
}

vec3f arc_road::point(const float t, const float offset, const vec3f &up) const
{
    vec3f pos;
    vec3f tan;

    float local;
    const size_t idx = locate_scale(t, offset, local);
    if(idx & 1)
    {
        const size_t real_idx = idx/2;
        circle_frame(pos, tan, local*arcs_[real_idx], frames_[real_idx], radii_[real_idx]);
        const vec3f left(tvmet::normalize(tvmet::cross(up, tan)));
        return vec3f(pos + left*offset);
    }
    else
    {
        const int real_idx = idx/2-1;

        if(real_idx < 0 || frames_.empty())
        {
            pos = points_.front();
            tan = normals_.front();
        }
        else
            circle_frame(pos, tan, arcs_[real_idx], frames_[real_idx], radii_[real_idx]);

        const vec3f left(tvmet::normalize(tvmet::cross(up, tan)));
        return vec3f(pos + left*offset + tan*local*feature_size(idx, offset));
    }
}

mat3x3f arc_road::frame(const float t, float offset, const bool reverse, const vec3f &up) const
{
    vec3f pos;
    vec3f tan;

    float local;
    const size_t idx = locate_scale(t, offset, local);
    if(idx & 1)
    {
        const size_t real_idx = idx/2;
        circle_frame(pos, tan, local*arcs_[real_idx], frames_[real_idx], radii_[real_idx]);
    }
    else
    {
        const int real_idx = idx/2-1;

        if(real_idx < 0 || frames_.empty())
        {
            pos = points_.front();
            tan = normals_.front();
        }
        else
            circle_frame(pos, tan, arcs_[real_idx], frames_[real_idx], radii_[real_idx]);

        pos += tan*local*feature_size(idx, offset);
    }

    if(reverse)
    {
        tan    *= -1;
        offset *= -1;
    }

    const vec3f left  (tvmet::normalize(tvmet::cross(up, tan)));
    const vec3f new_up(tvmet::normalize(tvmet::cross(tan, left)));

    pos += left*offset;

    mat3x3f res;
    res(0, 0) = tan[0]; res(0, 1) = left[0]; res(0, 2) = new_up[0];
    res(1, 0) = tan[1]; res(1, 1) = left[1]; res(1, 2) = new_up[1];
    res(2, 0) = tan[2]; res(2, 1) = left[2]; res(2, 2) = new_up[2];
    return res;
}

mat4x4f arc_road::point_frame(const float t, float offset, bool reverse, const vec3f &up) const
{
    vec3f pos;
    vec3f tan;

    float local;
    const size_t idx = locate_scale(t, offset, local);
    if(idx & 1)
    {
        const size_t real_idx = idx/2;
        circle_frame(pos, tan, local*arcs_[real_idx], frames_[real_idx], radii_[real_idx]);
    }
    else
    {
        const int real_idx = idx/2-1;

        if(real_idx < 0 || frames_.empty())
        {
            pos = points_.front();
            tan = normals_.front();
        }
        else
            circle_frame(pos, tan, arcs_[real_idx], frames_[real_idx], radii_[real_idx]);

        pos += tan*local*feature_size(idx, offset);
    }

    if(reverse)
    {
        tan    *= -1;
        offset *= -1;
    }

    const vec3f left  (tvmet::normalize(tvmet::cross(up, tan)));
    const vec3f new_up(tvmet::normalize(tvmet::cross(tan, left)));

    pos += left*offset;

    mat4x4f res;
    res(0, 0) = tan[0];res(0, 1) = left[0];res(0, 2) = new_up[0];res(0, 3) = pos[0];
    res(1, 0) = tan[1];res(1, 1) = left[1];res(1, 2) = new_up[1];res(1, 3) = pos[1];
    res(2, 0) = tan[2];res(2, 1) = left[2];res(2, 2) = new_up[2];res(2, 3) = pos[2];
    res(3, 0) = 0.0f;  res(3, 1) = 0.0f;   res(3, 2) = 0.0f;     res(3, 3) = 1.0f;
    return res;
}

void arc_road::translate(const vec3f &o)
{
    BOOST_FOREACH(vec3f &pt, points_)
    {
        pt += o;
    }

    BOOST_FOREACH(mat4x4f &fr, frames_)
    {
        for(int i = 0; i < 3; ++i)
            fr(i, 3) += o[i];
    }
}

void arc_road::bounding_box(vec3f &low, vec3f &high) const
{
    BOOST_FOREACH(const vec3f &pt, points_)
    {
        for(size_t i = 0; i < 3; ++i)
        {
            if(pt[i] < low[i])
                low[i] = pt[i];
            if(pt[i] > high[i])
                high[i] = pt[i];
        }
    }
}

float arc_road::parameter_map(const float t, const float offset) const
{
    const float blen = length(offset);
    if(blen > 0.0f)
        return length(t, offset)/length(offset);
    else
        return 0.0f;
}

float arc_road::length_at_feature(const size_t i, const float p, const float offset) const
{
    return feature_base(i, offset) + p*feature_size(i, offset);
}

void arc_road::extract_arc(std::vector<vertex> &result, const size_t i, const vec2f &i_range, const float offset, const float resolution, const vec3f &up) const
{
    const float resolution2 = resolution*resolution;

    vec2f in_range(i_range);
    in_range[0] = std::max(0.0f, in_range[0]);
    in_range[1] = std::min(1.0f, in_range[1]);

    assert(in_range[0] < in_range[1]);
    assert(in_range[0] >= 0.0f);
    assert(in_range[1] <= 1.0f);

    std::vector<std::pair<float, vertex> > new_points;

    // add last point
    {
        vec3f pos;
        vec3f tan;

        circle_frame(pos, tan, in_range[1]*arcs_[i], frames_[i], radii_[i]);
        const vec3f left(tvmet::normalize(tvmet::cross(up, tan)));
        const vec3f real_up(tvmet::normalize(tvmet::cross(tan, left)));
        const vertex vertex_end(vertex(vec3f(pos + left*offset), real_up, vec2f(in_range[1], 0.0f)));

        new_points.push_back(std::make_pair(in_range[1]*arcs_[i], vertex_end));
    }

    // add first point
    {
        vec3f pos;
        vec3f tan;

        circle_frame(pos, tan, in_range[0]*arcs_[i], frames_[i], radii_[i]);

        const vec3f left   (tvmet::normalize(tvmet::cross(up, tan)));
        const vec3f real_up(tvmet::normalize(tvmet::cross(tan, left)));
        const vertex vertex_start(vertex(vec3f(pos + left*offset), real_up, vec2f(in_range[0], 0.0f)));

        if(result.empty() || distance2(vertex_start.position, result.back().position) >= resolution2)
        {
            new_points.push_back(std::make_pair(in_range[0]*arcs_[i], vertex_start));
            result.push_back(vertex_start);
        }
        else
            new_points.push_back(std::make_pair(in_range[0]*arcs_[i], result.back()));
    }

    while(new_points.size() > 1)
    {
        const std::pair<float, vertex> &front = new_points[new_points.size()-1];
        const std::pair<float, vertex> &next  = new_points[new_points.size()-2];

        if (distance2(front.second.position, next.second.position) > resolution2)
        {
            //            assert(next.first - front.first > 1e-3);
            const float new_theta = (next.first + front.first)/2;
            vec3f pos;
            vec3f tan;
            circle_frame(pos, tan, new_theta, frames_[i], radii_[i]);

            const vec3f left(tvmet::normalize(tvmet::cross(up, tan)));
            const vec3f real_up(tvmet::normalize(tvmet::cross(tan, left)));
            assert(new_theta/arcs_[i] >= 0.0f);
            new_points.push_back(std::make_pair(new_theta, vertex(vec3f(pos + left*offset),
                                                                  real_up,
                                                                  vec2f(new_theta/arcs_[i], 0.0f))));
            std::swap(new_points[new_points.size()-1], new_points[new_points.size()-2]);
        }
        else
        {
            new_points.pop_back();
            if(distance2(new_points.back().second.position, result.back().position) >= 1e-2)
                result.push_back(new_points.back().second);
        }
    }
}

void arc_road::extract_center(std::vector<vertex> &result, const vec2f &in_range, const float offset, const float resolution, const vec3f &up) const
{
    const vec2f new_range(parameter_map(in_range[0], offset), parameter_map(in_range[1], offset));
    const size_t last = result.size();
    extract_line(result, new_range, offset, resolution, up);

    BOOST_FOREACH(vertex &v, std::make_pair(boost::next(result.begin(), last), result.end()))
    {
        v.tex_coord[0] = parameter_map(v.tex_coord[0], offset);
    }
}

template <typename T>
static inline T lerp(const T x, const T a, const T b)
{
    return x*(b-a)+a;
}

template <typename T>
static inline T unlerp(const T x, const T a, const T b)
{
    return (x-a)/(b-a);
}

static void rescale_tex_coords(const std::vector<vertex>::iterator &start, const std::vector<vertex>::iterator &end, const vec2f &dest_range, const vec2f &src_range)
{

    BOOST_FOREACH(vertex &v, std::make_pair(start, end))
    {
        v.tex_coord[0] = lerp(unlerp(v.tex_coord[0], src_range[0], src_range[1]),
                              dest_range[0], dest_range[1]);
        assert(v.tex_coord[0] >= 0.0f);
    }
}

void arc_road::extract_line(std::vector<vertex> &result, const vec2f &in_range, const float offset, const float resolution, const vec3f &up) const
{
    const float resolution2 = resolution*resolution;

    size_t input_start = result.size();
    const float len   = length(offset);

    vec2f range(in_range);
    bool reversed = range[0] > range[1];
    if(reversed)
        std::swap(range[0], range[1]);

    float        start_local;
    size_t       start_arc;
    const size_t start_feature = locate_scale(range[0], offset, start_local);

    float        end_local;
    const size_t end_feature   = locate_scale(range[1], offset, end_local);

    if(start_feature & 1)
    {
        const size_t last = result.size();
        if(start_feature == end_feature)
            extract_arc(result, start_feature/2, vec2f(start_local, end_local), offset, resolution, up);
        else
            extract_arc(result, start_feature/2, vec2f(start_local, 1.0f),      offset, resolution, up);
        start_arc = start_feature/2 + 1;

        const float fst = feature_base(start_feature, offset);
        const vec2f frange(fst, fst+feature_size(start_feature, offset));
        rescale_tex_coords(boost::next(result.begin(), last), result.end(), vec2f(frange/len), vec2f(0.0f, 1.0f));
    }
    else
    {
        vec3f pos, tan;
        const int real_idx = start_feature/2-1;
        if(real_idx < 0 || frames_.empty())
        {
            pos = points_.front();
            tan = normals_.front();
        }
        else
            circle_frame(pos, tan, arcs_[real_idx], frames_[real_idx], radii_[real_idx]);

        const float fst = feature_base(start_feature, offset);
        const vec2f frange(fst, fst+feature_size(start_feature, offset));

        const vec3f  left(tvmet::normalize(tvmet::cross(up, tan)));
        const vec3f  real_up(tvmet::normalize(tvmet::cross(tan, left)));
        const vertex start_vertex(vertex(vec3f(pos + left*offset + tan*start_local*feature_size(start_feature, offset)),
                                         real_up,
                                         vec2f(lerp(start_local, frange[0]/len, frange[1]/len), 0.0f)));
        assert(lerp(start_local, frange[0]/len, frange[1]/len) >= 0.0f);

        if(result.empty() || distance2(start_vertex.position, result.back().position) >= resolution2)
            result.push_back(start_vertex);

        start_arc = start_feature/2;
    }

    const size_t end_arc = end_feature/2;
    for(size_t i = start_arc; i < end_arc; ++i)
    {
        const float fst = feature_base(2*i+1, offset);
        const vec2f frange(fst, fst+feature_size(2*i+1, offset));

        const size_t last = result.size();
        extract_arc(result, i, vec2f(0.0, 1.0), offset, resolution, up);
        rescale_tex_coords(boost::next(result.begin(), last), result.end(), vec2f(frange/len), vec2f(0.0f, 1.0f));
    }

    if(end_feature & 1)
    {
        if(start_feature != end_feature)
        {
            const float fst = feature_base(end_feature, offset);
            const vec2f frange(fst, fst+feature_size(end_feature, offset));

            const size_t last = result.size();
            extract_arc(result, end_feature/2, vec2f(0.0, end_local), offset, resolution, up);
            rescale_tex_coords(boost::next(result.begin(), last), result.end(), vec2f(frange/len), vec2f(0.0f, 1.0f));
        }
    }
    else
    {
        vec3f pos, tan;
        const int real_idx = end_feature/2-1;
        if(real_idx < 0 || frames_.empty())
        {
            pos = points_.front();
            tan = normals_.front();
        }
        else
            circle_frame(pos, tan, arcs_[real_idx], frames_[real_idx], radii_[real_idx]);

        const float fst = feature_base(end_feature, offset);
        const vec2f frange(fst, fst+feature_size(end_feature, offset));

        const vec3f left(tvmet::normalize(tvmet::cross(up, tan)));
        const vec3f real_up(tvmet::normalize(tvmet::cross(tan, left)));
        const vertex end_vertex(vertex(vec3f(pos + left*offset + tan*end_local*feature_size(end_feature, offset)),
                                       real_up,
                                       vec2f(lerp(end_local, frange[0]/len, frange[1]/len), 0.0f)));
        assert(lerp(end_local, frange[0]/len, frange[1]/len) >= 0.0f);

        if(result.empty() || distance2(end_vertex.position, result.back().position) >= resolution2)
            result.push_back(end_vertex);
    }

    if(reversed)
    {
        std::vector<vertex>::iterator reverse_start = result.begin();
        reverse_start += input_start;
        std::reverse(reverse_start, result.end());
    }
}

void arc_road::make_mesh(std::vector<vertex> &vrts, std::vector<vec3u> &faces,
                         const vec2f &range,
                         const vec2f &offsets, const float resolution) const
{
    size_t r1 = vrts.size();
    extract_center(vrts, range, offsets[0], resolution);
    BOOST_FOREACH(vertex &v, std::make_pair(boost::next(vrts.begin(), r1), vrts.end()))
    {
        v.tex_coord[1] = 0.0f;
    }

    r1 = vrts.size();
    extract_center(vrts, vec2f(range[1], range[0]), offsets[1], resolution);
    BOOST_FOREACH(vertex &v, std::make_pair(boost::next(vrts.begin(), r1), vrts.end()))
    {
        v.tex_coord[1] = 1.0f;
    }

    ::make_mesh(faces, vrts, vec2i(0, r1), vec2i(vrts.size(), r1));
}

float arc_road::feature_base(const size_t i, const float offset) const
{
    const int seg_idx = i/2 + (i&1);
    const int arc_idx = i/2;

    const float res = seg_clengths_[seg_idx] + arc_clengths_[arc_idx][0] + offset*arc_clengths_[arc_idx][1];
    return std::max(res, 0.0f);
}

float arc_road::feature_size(const size_t i, const float offset) const
{
    assert(i < 2*frames_.size()+1);
    return feature_base(i+1, offset) - feature_base(i, offset);
}

size_t arc_road::locate(const float t, const float offset) const
{
    const float scaled_t = t*length(offset);

    size_t low = 0;
    size_t high = 2*frames_.size()+1;
    while (low < high)
    {
        const size_t mid = low + ((high - low) / 2);
        float lookup = feature_base(mid, offset);

        if (lookup < scaled_t)
            low = mid + 1;
        else
            high = mid;
    }
    if(low > 0)
        --low;
    while(low < 2*frames_.size() && feature_size(low, offset) == 0)
        ++low;

    return low;
}

size_t arc_road::locate_scale(const float t, const float offset, float &local) const
{
    const float scaled_t = t*length(offset);

    size_t low = 0;
    size_t high = 2*frames_.size()+1;
    while (low < high)
    {
        const size_t mid = low + ((high - low) / 2);
        float lookup = feature_base(mid, offset);

        if (lookup < scaled_t)
            low = mid + 1;
        else
            high = mid;
    }
    if(low > 0)
        --low;
    while(low < 2*frames_.size() && feature_size(low, offset) == 0)
        ++low;

    float lookup = feature_base(low, offset);
    float base   = feature_size(low, offset);

    local = (base > 0.0f) ? (scaled_t - lookup) / base : 0.0f;

    assert(local >= 0.0f);

    return low;
}

bool arc_road::check() const
{
    const size_t N_pts       = points_.size();
    return frames_.size()   == N_pts-2
    && radii_.size()        == N_pts-2
    && arcs_.size()         == N_pts-2
    && seg_clengths_.size() == N_pts
    && arc_clengths_.size() == N_pts-1
    && normals_.size()      == N_pts-1;
}

str  arc_road::svg_arc_path_center(const vec2f &interval, const float offset, const bool start) const
{
    const vec2f new_range(parameter_map(interval[0], offset), parameter_map(interval[1], offset));
    return svg_arc_path(new_range, offset, start);
}

str arc_road::svg_arc_path(const vec2f &interval, const float offset, const bool start) const
{
    const vec3f up(0.0, 0.0, 1.0);

    vec2f range(interval);
    bool reversed = range[0] > range[1];
    if(reversed)
        std::swap(range[0], range[1]);

    float        start_local;
    size_t       start_arc;
    const size_t start_feature = locate_scale(range[0], offset, start_local);

    float        end_local;
    const size_t end_feature   = locate_scale(range[1], offset, end_local);

    path p;
    vec3f last_point;
    if(start_feature & 1)
    {
        float end;
        if(start_feature == end_feature)
            end = end_local;
        else
            end = 1.0f;

        vec2f in_range(std::max(0.0f, start_local),
                       std::min(1.0f, end));

        vec3f start;
        {
            vec3f pos;
            vec3f tan;
            circle_frame(pos, tan, in_range[0]*arcs_[start_feature/2], frames_[start_feature/2], radii_[start_feature/2]);

            const vec3f left   (tvmet::normalize(tvmet::cross(up, tan)));
            const vec3f real_up(tvmet::normalize(tvmet::cross(tan, left)));
            start = pos + left*offset;
        }
        {
            vec3f pos;
            vec3f tan;
            circle_frame(pos, tan, in_range[1]*arcs_[start_feature/2], frames_[start_feature/2], radii_[start_feature/2]);
            const vec3f left   (tvmet::normalize(tvmet::cross(up, tan)));
            const vec3f real_up(tvmet::normalize(tvmet::cross(tan, left)));
            last_point = pos + left*offset;
        }

        p.add_arc(start, radii_[start_feature/2], offset, (frames_[start_feature/2](2,2) < 0), last_point);

        start_arc = start_feature/2 + 1;
    }
    else
    {
        vec3f pos, tan;
        const int real_idx = start_feature/2-1;
        if(real_idx < 0 || frames_.empty())
        {
            pos = points_.front();
            tan = normals_.front();
        }
        else
            circle_frame(pos, tan, arcs_[real_idx], frames_[real_idx], radii_[real_idx]);

        const vec3f left(tvmet::normalize(tvmet::cross(up, tan)));
        const vec3f real_up(tvmet::normalize(tvmet::cross(tan, left)));
        last_point = vec3f(pos + left*offset + tan*start_local*feature_size(start_feature, offset));

        start_arc = start_feature/2;
    }

    const size_t end_arc = end_feature/2;
    for(size_t i = start_arc; i < end_arc; ++i)
    {
        vec3f point0;
        {
            vec3f pos;
            vec3f tan;
            circle_frame(pos, tan, 0, frames_[i], radii_[i]);
            const vec3f left(tvmet::normalize(tvmet::cross(up, tan)));
            const vec3f real_up(tvmet::normalize(tvmet::cross(tan, left)));
            point0 = vec3f(pos + left*offset);
        }

        vec3f point1;
        {
            vec3f pos;
            vec3f tan;
            circle_frame(pos, tan, arcs_[i], frames_[i], radii_[i]);
            const vec3f left(tvmet::normalize(tvmet::cross(up, tan)));
            const vec3f real_up(tvmet::normalize(tvmet::cross(tan, left)));
            point1 = vec3f(pos + left*offset);
        }

        p.add_line(last_point, point0);
        last_point = point1;
        p.add_arc(point0, radii_[i], offset, (frames_[i](2,2) < 0), point1);
    }

    if(end_feature & 1)
    {
        if(start_feature != end_feature)
        {
            vec3f point0;
            {
                vec3f pos;
                vec3f tan;
                circle_frame(pos, tan, 0, frames_[end_feature/2], radii_[end_feature/2]);
                const vec3f left(tvmet::normalize(tvmet::cross(up, tan)));
                const vec3f real_up(tvmet::normalize(tvmet::cross(tan, left)));
                point0 = vec3f(pos + left*offset);
            }

            vec3f point1;
            {
                vec3f pos;
                vec3f tan;
                circle_frame(pos, tan, end_local*arcs_[end_feature/2], frames_[end_feature/2], radii_[end_feature/2]);
                const vec3f left(tvmet::normalize(tvmet::cross(up, tan)));
                const vec3f real_up(tvmet::normalize(tvmet::cross(tan, left)));
                point1 = vec3f(pos + left*offset);
            }

            p.add_line(last_point, point0);
            last_point = point1;
            p.add_arc(point0, radii_[end_feature/2], offset, (frames_[end_feature/2](2,2) < 0), point1);
        }
    }
    else
    {
        vec3f pos, tan;
        const int real_idx = end_feature/2-1;
        if(real_idx < 0 || frames_.empty())
        {
            pos = points_.front();
            tan = normals_.front();
        }
        else
            circle_frame(pos, tan, arcs_[real_idx], frames_[real_idx], radii_[real_idx]);

        const vec3f left(tvmet::normalize(tvmet::cross(up, tan)));
        const vec3f real_up(tvmet::normalize(tvmet::cross(tan, left)));
        const vec3f end(pos + left*offset + tan*end_local*feature_size(end_feature, offset));
        p.add_line(last_point, end);
    }

    if(reversed)
        p.reverse();

    p.remove_duplicates();
    return p.stringify(start);
}

str arc_road::svg_arc_arc_path(const size_t i) const
{
    assert(i > 0 && i < normals_.size());

    const vec3f c(center(i));
    vec3f start,tan;
    circle_frame(start, tan, 0, frames_[i-1], radii_[i-1]);

    vec3f end;
    circle_frame(end, tan, arcs_[i-1], frames_[i-1], radii_[i-1]);

    return boost::str(boost::format("M%f,%f L%f,%f A%f,%f 0 0,%d %f,%f z") % c[0] % c[1] % start[0] % start[1] % radii_[i-1] % radii_[i-1] % (frames_[i-1](2,2) < 0) % end[0] % end[1]);
}

str arc_road::svg_poly_path_center(const vec2f &interval, const float offset, const bool continuation) const
{
    polyline_road pr;
    pr.points_ = points_;
    pr.initialize();

    return pr.svg_poly_path_center(interval, offset, continuation);
}

str arc_road::svg_poly_path(const vec2f &interval, const float offset, const bool continuation) const
{
    polyline_road pr;
    pr.points_ = points_;
    pr.initialize();

    return pr.svg_poly_path(interval, offset, continuation);
}

bool projection_intersect(vec3f &result,
                          const vec3f &o0, const vec3f &n0,
                          const vec3f &o1, const vec3f &n1)
{
    vec3f od(o1 - o0);
    od[2] = 0.0f;
    const float denom = -n0[0] * n1[1] + n0[1]*n1[0];
    if(length2(od) < 1e-6 || std::abs(denom) < 1e-6)
            return false;
    const float t0 = (-n1[1]*od[0] + n1[0]*od[1])/denom;
    const float t1 = (-n0[1]*od[0] + n0[0]*od[1])/denom;
    if(t0 >= 2.0f && t1 >= 2.0f)
    {
        result = vec3f(o0 + n0*t0);
        result[2] = (o0[2] + o1[2])/2;
        return true;
    }

    return false;
}

std::vector<vec3f> from_tan_pairs(const vec3f &start_point,
                                  const vec3f &start_tan,
                                  const vec3f &end_point,
                                  const vec3f &end_tan)
{

    std::vector<vec3f> pts;
    pts.push_back(start_point);

    vec3f middle;
    const bool okay(projection_intersect(middle,
                                         start_point, start_tan,
                                         end_point,   end_tan));
    if(!okay)
    {
        pts.push_back(vec3f(start_point + start_tan*4));
        pts.push_back(vec3f(end_point   + end_tan*4));
    }
    else
        pts.push_back(middle);

    pts.push_back(end_point);

    return pts;
}
