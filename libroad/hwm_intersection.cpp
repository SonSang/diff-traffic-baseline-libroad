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

    void intersection::translate(const vec3f &o)
    {
        BOOST_FOREACH(vec3f &pt, shape)
        {
            pt += o;
        }
        center += o;
    }

    static inline bool rightturn(const vec3f &pt0, const vec3f &pt1, const vec3f &pt2, float eps=-1e-6)
    {
        const float rtval = ((pt1[0]-pt0[0])*(pt2[1]-pt1[1]) - (pt2[0]-pt1[0])*(pt1[1]-pt0[1]));
        return rtval < eps;
    }

    static inline bool lt(float l, float r, float eps=-1e-6)
    {
        return (l - r) < eps;
    }

    static inline bool eq(float l, float r, float eps=1e-6)
    {
        return std::abs(r - l) < eps;
    }

    struct lexicographic
    {
        inline bool operator()(const vec3f &pt0, const vec3f &pt1) const
        {
            return lt(pt0[0], pt1[0]) || (eq(pt0[0], pt1[0]) && lt(pt0[1], pt1[1]));
        }
    };

    static inline void convex_hull(std::vector<vec3f> &pts)
    {
        lexicographic lx;
        std::sort(pts.begin(), pts.end(), lx);

        std::vector<vec3f> newpts;
        newpts.push_back(pts[0]);
        int count = 1;
        while(count < static_cast<int>(pts.size()))
        {
            const vec3f &bpt = newpts.back();
            if(!(eq(bpt[0], pts[count][0]) && eq(bpt[1], pts[count][1])))
                newpts.push_back(pts[count]);
            ++count;
        }
        pts.clear();

        pts.push_back(newpts[0]);
        pts.push_back(newpts[1]);

        for(count = 2; count < static_cast<int>(newpts.size()); ++count)
        {
            pts.push_back(newpts[count]);

            int back = static_cast<int>(pts.size())-1;
            while(back > 1 && !rightturn(pts[back-2], pts[back-1], pts[back]))
            {
                std::swap(pts[back], pts[back-1]);
                pts.pop_back();
                --back;
            }
        }

        int upper_back = static_cast<int>(pts.size());
        for(count = static_cast<int>(newpts.size())-2; count >= 0; --count)
        {
            pts.push_back(newpts[count]);

            int back = static_cast<int>(pts.size())-1;
            while(back > upper_back && !rightturn(pts[back-2], pts[back-1], pts[back]))
            {
                std::swap(pts[back], pts[back-1]);
                pts.pop_back();
                --back;
            }
        }
        pts.pop_back();
    }

    void intersection::build_shape(float lane_width)
    {
        shape.reserve(2*(outgoing.size()+incoming.size()));

        BOOST_FOREACH(const lane *lid, incoming)
        {
            shape.push_back(lid->point(1.0, -0.5*lane_width));
            shape.push_back(lid->point(1.0, +0.5*lane_width));
        }

        BOOST_FOREACH(const lane *lid, outgoing)
        {
            shape.push_back(lid->point(0.0, -0.5*lane_width));
            shape.push_back(lid->point(0.0, +0.5*lane_width));
        }

        convex_hull(shape);

        center = vec3f(0.0f);
        BOOST_FOREACH(const vec3f &pt, shape)
        {
            center += pt;
        }
        center /= shape.size();
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
