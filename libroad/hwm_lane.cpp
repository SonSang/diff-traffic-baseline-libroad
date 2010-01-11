#include "hwm_network.hpp"

namespace hwm
{
    bool lane::terminus::check(bool start, const lane *parent) const
    {
        if(!inters)
            return intersect_in_ref == -1;
        if(inters->id.empty() || intersect_in_ref == -1)
            return false;
        const std::vector<lane*> &cont = start ? inters->outgoing : inters->incoming;
        return cont[intersect_in_ref] == parent;
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

    vec3f   lane::road_membership::point      (float t, const vec3f &up) const
    {
        if(interval[0] > interval[1])
            t = 1 - t;
        return parent_road->rep.point(t, lane_position, up);
    }

    mat3x3f lane::road_membership::frame      (float t, const vec3f &up) const
    {
        bool reversed = (interval[0] > interval[1]);
        if(reversed)
            t = 1 - t;
        mat3x3f res(parent_road->rep.frame(t, lane_position, up));

        if(reversed)
        {//flip res
        }
        return res;
    }

    mat4x4f lane::road_membership::point_frame(float t, const vec3f &up) const
    {
        bool reversed = (interval[0] > interval[1]);
        if(reversed)
            t = 1 - t;
        mat4x4f res(parent_road->rep.point_frame(t*std::abs(interval[0]-interval[1])+interval[0], lane_position, up));

        if(reversed)
        {//flip res
        }
        return res;
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
           !start.check(true, this) || !end.check(false, this) ||
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

    void lane::scale_offsets(const float lane_width)
    {
        BOOST_FOREACH(road_membership::intervals::entry &rmie, road_memberships)
        {
            rmie.second.scale_offsets(lane_width);
        }
    }

    float   lane::length     () const
    {
        float total = 0.0f;
        BOOST_FOREACH(const road_membership::intervals::entry &rmie, road_memberships)
        {
            total += rmie.second.length();
        }

        return total;
    }

    vec3f   lane::point      (float t, const vec3f &up) const
    {
        float local;
        road_membership::intervals::const_iterator rmici = road_memberships.find_rescale(t, local);
        return rmici->second.point(local, up);
    }

    mat3x3f lane::frame      (float t, const vec3f &up) const
    {
        float local;
        road_membership::intervals::const_iterator rmici = road_memberships.find_rescale(t, local);
        return rmici->second.frame(local, up);
    }

    mat4x4f lane::point_frame(float t, const vec3f &up) const
    {
        float local;
        road_membership::intervals::const_iterator rmici = road_memberships.find_rescale(t, local);
        return rmici->second.point_frame(local, up);
    }
}
