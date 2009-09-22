#include "hwm_network.hpp"

namespace hwm
{
    bool road::check() const
    {
        return !id.empty() && rep.check();
    }

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

    bool intersection::check() const
    {
        if(id.empty())
            return false;

        int count = 0;
        BOOST_FOREACH(const lane* lp, incoming)
        {
            if(!lp || lp->end.inters != this || lp->end.intersect_in_ref != count)
                return false;
            ++count;
        }

        count = 0;
        BOOST_FOREACH(const lane* lp, outgoing)
        {
            if(!lp || lp->start.inters != this || lp->start.intersect_in_ref != count)
                return false;
            ++count;
        }

        BOOST_FOREACH(const intersection::state &s, states)
        {
            if(s.duration <= 0.0f)
                return false;
            if(s.in_states.size() != incoming.size())
                return false;
            if(s.out_states.size() != outgoing.size())
                return false;
            int c = 0;
            BOOST_FOREACH(const intersection::state::out_id &oid, s.in_states)
            {
                if(oid.out_ref != -1)
                    if(s.out_states[oid.out_ref].in_ref != c)
                        return false;
                ++c;
            }
            c = 0;
            BOOST_FOREACH(const intersection::state::in_id &iid, s.out_states)
            {
                if(iid.in_ref != -1)
                    if(s.in_states[iid.in_ref].out_ref != c)
                        return false;
                ++c;
            }
        }

        return true;
    }

    bool network::check() const
    {
        if(gamma <= 0.0f || gamma >= 1.0f)
            return false;

        typedef strhash<road>::type::value_type rval;
        BOOST_FOREACH(const rval &r, roads)
        {
            if(r.first != r.second.id || !r.second.check())
                return false;
        }

        typedef strhash<lane>::type::value_type lval;
        BOOST_FOREACH(const lval &l, lanes)
        {
            if(l.first != l.second.id || !l.second.check())
                return false;
        }

        typedef strhash<intersection>::type::value_type ival;
        BOOST_FOREACH(const ival &i, intersections)
        {
            if(i.first != i.second.id || !i.second.check())
                return false;
        }

        return true;
    }
}
