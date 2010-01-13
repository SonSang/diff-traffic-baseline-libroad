#include "hwm_network.hpp"

namespace hwm
{
    bool intersection::state::state_pair::check(const intersection &parent) const
    {
        return fict_lane && in_idx >= 0 && in_idx < static_cast<int>(parent.incoming.size())
        && out_idx >= 0 && out_idx < static_cast<int>(parent.outgoing.size());
    }

    bool intersection::state::check(const intersection &parent) const
    {
        if(duration <= 0.0f)
            return false;

        BOOST_FOREACH(const intersection::state::state_pair &sp, in_pair())
        {
            if(!sp.check(parent))
                return false;
        }

        return true;
    }

    intersection::state::state_pair_in &intersection::state::in_pair()
    {
        return state_pairs.get<intersection::state::in>();
    }

    const intersection::state::state_pair_in &intersection::state::in_pair() const
    {
        return state_pairs.get<intersection::state::in>();
    }

    intersection::state::state_pair_out &intersection::state::out_pair()
    {
        return state_pairs.get<intersection::state::out>();
    }

    const intersection::state::state_pair_out &intersection::state::out_pair() const
    {
        return state_pairs.get<intersection::state::out>();
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

        const state::state_pair_in &in_pairs = states[current_state].state_pairs.get<state::in>();
        state::state_pair_in::const_iterator result = in_pairs.find(incoming_ref);

        return (result == in_pairs.end()) ? 0 : result->fict_lane;
    }

    lane *intersection::upstream_lane(const int outgoing_ref) const
    {
        assert(current_state >= 0 && current_state < static_cast<int>(states.size()));

        const state::state_pair_out &out_pairs = states[current_state].state_pairs.get<state::out>();
        state::state_pair_out::const_iterator result = out_pairs.find(outgoing_ref);

        return (result == out_pairs.end()) ? 0 : result->fict_lane;
    }
}
