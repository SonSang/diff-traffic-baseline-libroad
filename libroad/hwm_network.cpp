#include "hwm_network.hpp"

namespace hwm
{
    bool road::check() const
    {
        return !id.empty() && rep.check();
    }

    bool lane::terminus::check() const
    {
        return !inters || !(inters->id.empty());
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
           !start.check() || !end.check() ||
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

        BOOST_FOREACH(const lane* lp, incoming)
        {
            if(!lp)
                return false;
        }

        BOOST_FOREACH(const lane* lp, outgoing)
        {
            if(!lp)
                return false;
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
