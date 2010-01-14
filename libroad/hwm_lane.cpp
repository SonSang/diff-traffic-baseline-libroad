#include "hwm_network.hpp"

namespace hwm
{
    lane::lane() : start(0), end(0)
    {}

    lane::lane(const lane &l) : id(l.id),
                                road_memberships(l.road_memberships),
                                left(l.left),
                                right(l.right),
                                speedlimit(l.speedlimit)
    {
        start = l.start ? l.start->clone() : 0;
        end   = l.end   ? l.end->clone()   : 0;
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

    void lane::make_mesh(std::vector<vertex> &verts, std::vector<vec3u> &faces, const float lane_width, const float resolution) const
    {
        int start_high = static_cast<int>(verts.size());

        std::vector<vertex> new_verts;
        typedef hwm::lane::road_membership::intervals::entry rme;
        BOOST_FOREACH(const rme &rm_entry, road_memberships)
        {
            const hwm::lane::road_membership &rm = rm_entry.second;
            rm.parent_road->rep.extract_center(new_verts, rm.interval, rm.lane_position-lane_width*0.5, resolution);
        }
        int end_high = static_cast<int>(verts.size() + new_verts.size());
        typedef hwm::lane::road_membership::intervals::const_reverse_iterator rme_it;
        for(rme_it current = road_memberships.rbegin(); current != road_memberships.rend(); ++current)
        {
            const hwm::lane::road_membership &rm = current->second;
            const vec2f rev_interval(rm.interval[1], rm.interval[0]);
            rm.parent_road->rep.extract_center(new_verts, rev_interval, rm.lane_position+lane_width*0.5, resolution);
        }

        verts.insert(verts.end(), new_verts.begin(), new_verts.end());

        ::make_mesh(faces, verts, vec2i(start_high, end_high), vec2i(verts.size(), end_high));
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
