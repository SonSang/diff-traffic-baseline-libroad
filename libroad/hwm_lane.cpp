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

    bool lane::road_membership::empty() const
    {
        return !parent_road;
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
}
