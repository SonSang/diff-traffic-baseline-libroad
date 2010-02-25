#include "hwm_network.hpp"

namespace hwm
{
    lane::lane() : start(0), end(0), active(false)
    {}

    lane::lane(const lane &l) : id(l.id),
                                road_memberships(l.road_memberships),
                                left(l.left),
                                right(l.right),
                                speedlimit(l.speedlimit),
                                active(l.active),
                                user_datum(l.user_datum)
    {
        start = l.start ? l.start->clone() : 0;
        end   = l.end   ? l.end->clone()   : 0;
    }

    void lane::terminus::update_pointers(network &n)
    {
    }

    lane::terminus* lane::terminus::clone() const
    {
        return new lane::terminus();
    }

    bool lane::terminus::check(bool start, const lane *parent) const
    {
        return true;
    }

    lane* lane::terminus::incident(bool start) const
    {
        return 0;
    }

    void lane::intersection_terminus::update_pointers(network &n)
    {
        const intersection_map::iterator mine = n.intersections.find(adjacent_intersection->id);
        assert(mine != n.intersections.end());
        assert(mine->first == adjacent_intersection->id);
        adjacent_intersection                                  = &(mine->second);
    }

    lane::intersection_terminus* lane::intersection_terminus::clone() const
    {
        return new lane::intersection_terminus(adjacent_intersection, intersect_in_ref);
    }

    bool lane::intersection_terminus::check(bool start, const lane *parent) const
    {
        if(!adjacent_intersection || intersect_in_ref < 0)
            return false;

        const std::vector<lane*> &cont = start ? adjacent_intersection->outgoing : adjacent_intersection->incoming;
        return intersect_in_ref < static_cast<int>(cont.size()) && cont[intersect_in_ref] == parent;
    }

    lane* lane::intersection_terminus::incident(bool start) const
    {
        assert(adjacent_intersection);
        if(start)
            return adjacent_intersection->upstream_lane(intersect_in_ref);
        else
            return adjacent_intersection->downstream_lane(intersect_in_ref);
    }

    void lane::lane_terminus::update_pointers(network &n)
    {
        const lane_map::iterator mine = n.lanes.find(adjacent_lane->id);
        assert(mine != n.lanes.end());
        assert(mine->first == adjacent_lane->id);
        adjacent_lane = &(mine->second);
    }

    lane::lane_terminus* lane::lane_terminus::clone() const
    {
        return new lane::lane_terminus(adjacent_lane);
    }

    bool lane::lane_terminus::check(bool start, const lane *parent) const
    {
        if(!adjacent_lane)
            return false;

        const lane_terminus *other = dynamic_cast<const lane_terminus*>(start ? adjacent_lane->end : adjacent_lane->start);
        return other && other->adjacent_lane == parent;
    }

    lane* lane::lane_terminus::incident(bool start) const
    {
        assert(adjacent_lane);
        return adjacent_lane;
    }

    bool lane::road_membership::check() const
    {
        return (!empty() &&
                !parent_road->id.empty());
    }

    void lane::road_membership::scale_offsets(const float lane_width)
    {
        if(lane_position < 0)
            lane_position +=  0.5f;
        else if(lane_position > 0)
            lane_position += -0.5f;
        lane_position *= lane_width;
    }

    bool lane::road_membership::empty() const
    {
        return !parent_road;
    }

    float   lane::road_membership::length     () const
    {
        return std::abs(parent_road->rep.length(interval[0], lane_position) -
                        parent_road->rep.length(interval[1], lane_position));
    }

    vec3f   lane::road_membership::point      (float t, const float offset, const vec3f &up) const
    {
        t = t*(interval[1]-interval[0])+interval[0];
        return parent_road->rep.point(t, lane_position+offset, up);
    }

    mat3x3f lane::road_membership::frame      (float t, const vec3f &up) const
    {
        const bool reversed = (interval[0] > interval[1]);
        t = t*(interval[1]-interval[0])+interval[0];
        return parent_road->rep.frame(t, lane_position, reversed, up);
    }

    mat4x4f lane::road_membership::point_frame(float t, const float offset, const vec3f &up) const
    {
        const bool reversed = (interval[0] > interval[1]);
        t = t*(interval[1]-interval[0])+interval[0];
        return parent_road->rep.point_frame(t, lane_position+offset, reversed, up);
    }

    bool lane::adjacency::check() const
    {
        // could enforce symmetry here, but probably not necessary
        return !neighbor || !(neighbor->id.empty());
    }

    bool lane::adjacency::empty() const
    {
        return !neighbor;
    }

    bool lane::check() const
    {
        if(id.empty() ||
           road_memberships.empty() ||
           !start->check(true, this) || !end->check(false, this) ||
           speedlimit <= 0.0f)
            return false;

        BOOST_FOREACH(const road_membership::intervals::entry &rmie, road_memberships)
        {
            if(!rmie.second.check())
                return false;
        }

        BOOST_FOREACH(const adjacency::intervals::entry &aie, left)
        {
            if(!aie.second.check())
                return false;
        }

        BOOST_FOREACH(const adjacency::intervals::entry &aie, right)
        {
            if(!aie.second.check())
                return false;
        }

        return true;
    }

    void lane::auto_scale_memberships()
    {
        std::vector<float>  mlengths;
        mlengths.push_back(0);
        BOOST_FOREACH(const road_membership::intervals::entry &rmie, road_memberships)
        {
            mlengths.push_back(rmie.second.length() + mlengths.back());
        }

        const float inv_len = 1.0f/mlengths.back();
        road_membership::intervals new_rm;

        std::vector<float>::iterator current_len = mlengths.begin();
        road_membership::intervals::const_iterator it = road_memberships.begin();
        for(; current_len != mlengths.end() && it != road_memberships.end();
            ++current_len, ++it)
        {
            new_rm.insert(inv_len*(*current_len), it->second);
        }

        road_memberships = new_rm;
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
        }
    }

    void lane::make_mesh(std::vector<vertex> &verts, std::vector<vec3u> &faces, const float lane_width, const float resolution) const
    {
        const int start_high = static_cast<int>(verts.size());

        std::vector<vertex> new_verts;
        typedef road_membership::intervals::const_iterator rm_it;
        for(rm_it current = road_memberships.begin(); current != road_memberships.end(); ++current)
        {
            const road_membership &rm = current->second;
            const size_t last = new_verts.size();
            rm.parent_road->rep.extract_center(new_verts, rm.interval, rm.lane_position-lane_width*0.5, resolution);
            rescale_tex_coords(boost::next(new_verts.begin(), last), new_verts.end(), road_memberships.containing_interval(current), rm.interval);
        }

        verts.insert(verts.end(), new_verts.begin(), new_verts.end());
        new_verts.clear();

        const int end_high = static_cast<int>(verts.size());
        typedef road_membership::intervals::const_reverse_iterator rm_it_r;
        for(rm_it_r current = road_memberships.rbegin(); current != road_memberships.rend(); ++current)
        {
            const road_membership &rm = current->second;
            const size_t last = new_verts.size();
            const vec2f rev_interval(rm.interval[1], rm.interval[0]);
            rm.parent_road->rep.extract_center(new_verts, rev_interval, rm.lane_position+lane_width*0.5, resolution);
            rescale_tex_coords(boost::next(new_verts.begin(), last), new_verts.end(), road_memberships.containing_interval(current), rm.interval);
        }

        verts.insert(verts.end(), new_verts.begin(), new_verts.end());

        BOOST_FOREACH(vertex &v, std::make_pair(boost::next(verts.begin(), start_high), boost::next(verts.begin(), start_high)))
        {
            v.tex_coord[1] = 0.0f;
        }
        BOOST_FOREACH(vertex &v, std::make_pair(boost::next(verts.begin(), end_high), verts.end()))
        {
            v.tex_coord[1] = 1.0f;
        }

        ::make_mesh(faces, verts, vec2i(start_high, end_high), vec2i(verts.size(), end_high));
    }

    path lane::svg_arc_path(const float lane_width) const
    {
        path res;
        typedef road_membership::intervals::const_iterator rm_it;
        for(rm_it current = road_memberships.begin(); current != road_memberships.end(); ++current)
        {
            const road_membership &rm = current->second;
            res.append(rm.parent_road->rep.svg_arc_path_center(rm.interval, rm.lane_position-lane_width*0.5));
        }

        typedef road_membership::intervals::const_reverse_iterator rm_it_r;
        for(rm_it_r current = road_memberships.rbegin(); current != road_memberships.rend(); ++current)
        {
            const road_membership &rm = current->second;
            const vec2f rev_interval(rm.interval[1], rm.interval[0]);
            res.append(rm.parent_road->rep.svg_arc_path_center(rev_interval, rm.lane_position+lane_width*0.5));
        }

        return res;
    }

    path lane::svg_poly_path(const float lane_width) const
    {
        path res;
        typedef road_membership::intervals::const_iterator rm_it;
        for(rm_it current = road_memberships.begin(); current != road_memberships.end(); ++current)
        {
            const road_membership &rm = current->second;
            res.append(rm.parent_road->rep.svg_poly_path_center(rm.interval, rm.lane_position-lane_width*0.5));
        }

        typedef road_membership::intervals::const_reverse_iterator rm_it_r;
        for(rm_it_r current = road_memberships.rbegin(); current != road_memberships.rend(); ++current)
        {
            const road_membership &rm = current->second;
            const vec2f rev_interval(rm.interval[1], rm.interval[0]);
            res.append(rm.parent_road->rep.svg_poly_path_center(rev_interval, rm.lane_position+lane_width*0.5));
        }

        return res;
    }

    float lane::length     () const
    {
        float total = 0.0f;
        BOOST_FOREACH(const road_membership::intervals::entry &rmie, road_memberships)
        {
            total += rmie.second.length();
        }

        return total;
    }

    vec3f   lane::point      (float t, const float offset, const vec3f &up) const
    {
        float local;
        road_membership::intervals::const_iterator rmici = road_memberships.find_rescale(t, local);
        return rmici->second.point(local, offset, up);
    }

    mat3x3f lane::frame      (float t, const vec3f &up) const
    {
        float local;
        road_membership::intervals::const_iterator rmici = road_memberships.find_rescale(t, local);
        return rmici->second.frame(local, up);
    }

    mat4x4f lane::point_frame(float t, const float offset, const vec3f &up) const
    {
        float local;
        road_membership::intervals::const_iterator rmici = road_memberships.find_rescale(t, local);
        return rmici->second.point_frame(local, offset, up);
    }

    lane *lane::left_adjacency(float &param) const
    {
        float local;
        adjacency::intervals::const_iterator adii = left.find_rescale(param, local);
        if(adii == left.end())
            return 0;

        const adjacency &adj = adii->second;
        if(adj.neighbor)
            param = local * (adj.neighbor_interval[1] - adj.neighbor_interval[0]) + adj.neighbor_interval[0];
        return adj.neighbor;
    }

    lane *lane::right_adjacency(float &param) const
    {
        float local;
        adjacency::intervals::const_iterator adii = right.find_rescale(param, local);
        if(adii == right.end())
            return 0;

        const adjacency &adj = adii->second;
        if(adj.neighbor)
            param = local * (adj.neighbor_interval[1] - adj.neighbor_interval[0]) + adj.neighbor_interval[0];
        return adj.neighbor;
    }

    lane *lane::upstream_lane()   const
    {
        assert(start);
        return start->incident(true);
    }

    lane *lane::downstream_lane() const
    {
        assert(end);
        return end->incident(false);
    }
}
