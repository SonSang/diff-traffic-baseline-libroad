#include "hwm_network.hpp"

namespace hwm
{
    bool intersection::state::check(const intersection &parent) const
    {
        if(duration <= 0.0f)
            return false;
        if(in_states.size() != parent.incoming.size())
            return false;
        if(out_states.size() != parent.outgoing.size())
            return false;
        int c = 0;
        BOOST_FOREACH(const intersection::state::out_id &oid, in_states)
        {
            if(oid.out_ref != -1)
                if(out_states[oid.out_ref].in_ref != c)
                    return false;
            ++c;
        }
        c = 0;
        BOOST_FOREACH(const intersection::state::in_id &iid, out_states)
        {
            if(iid.in_ref != -1)
                if(in_states[iid.in_ref].out_ref != c)
                    return false;
            ++c;
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
            if(!lp)
                return false;
            const lane::intersection_terminus *it = dynamic_cast<const lane::intersection_terminus*>(lp->end);
            if(!it || it->adjacent_intersection != this || it->intersect_in_ref != count)
                return false;

            ++count;
        }

        count = 0;
        BOOST_FOREACH(const lane* lp, outgoing)
        {
            if(!lp)
                return false;
            const lane::intersection_terminus *it = dynamic_cast<const lane::intersection_terminus*>(lp->start);
            if(!it || it->adjacent_intersection != this || it->intersect_in_ref != count)
                return false;

            ++count;
        }

        BOOST_FOREACH(const intersection::state &s, states)
        {
            if(!s.check(*this))
                return false;
        }

        return true;
    }

    lane *intersection::downstream_lane(const int incoming_ref) const
    {
        assert(current_state >= 0 && current_state < static_cast<int>(states.size()));
        const intersection::state::out_id &out = states[current_state].in_states[incoming_ref];
        return (out.out_ref == -1) ? 0 : out.fict_lane;
    }

    lane *intersection::upstream_lane(const int outgoing_ref) const
    {
        assert(current_state >= 0 && current_state < static_cast<int>(states.size()));
        const intersection::state::in_id &in = states[current_state].out_states[outgoing_ref];
        return (in.in_ref == -1) ? 0 : in.fict_lane;
    }
}
