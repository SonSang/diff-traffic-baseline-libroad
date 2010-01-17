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

        bool first = fict_lanes.begin()->second.active;
        BOOST_FOREACH(const lane_pair &lp, fict_lanes)
        {
            if(lp.second.active != first)
                return false;
        }

        return true;
    }

    void intersection::state::translate(const vec3f &o)
    {
        BOOST_FOREACH(road_pair &frp, fict_roads)
        {
            frp.second.translate(o);
        }
    }

    static inline bool projection_intersect(vec3f &result,
                                            const vec3f &o0, const vec3f &n0,
                                            const vec3f &o1, const vec3f &n1)
    {
        vec3f od(o1 - o0);
        od[2] = 0.0f;
        const float denom = -n0[0] * n1[1] + n0[1]*n1[0];
        if(tvmet::dot(od, od) < 1e-6 || std::abs(denom) < 1e-6)
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

    void intersection::state::build_fictitious_lanes(const intersection &parent)
    {
        intersection::state::state_pair_in::iterator current = in_pair().begin();
        for(; current != in_pair().end(); ++current)
        {
            const intersection::state::state_pair &sp = *current;
            lane *in  = parent.incoming[sp.in_idx];
            lane *out = parent.outgoing[sp.out_idx];

            const str road_id(boost::str(boost::format("%s_to_%s_fict_road") % in->id % out->id));

            road_map::iterator new_road_itr(fict_roads.find(road_id));
            assert(new_road_itr == fict_roads.end());

            new_road_itr = fict_roads.insert(new_road_itr, std::make_pair(road_id, road()));

            road &new_road = new_road_itr->second;
            new_road.name        = road_id;
            new_road.id          = road_id;

            vec3f start_point;
            vec3f start_tan;
            vec3f end_point;
            vec3f end_tan;
            {
                const mat4x4f start(in ->point_frame(1.0));
                const mat4x4f end  (out->point_frame(0.0));
                for(int i = 0; i < 3; ++i)
                {
                    start_point[i] = start(i, 3);
                    start_tan[i]   = start(i, 0);
                    end_point[i]   = end(i, 3);
                    end_tan[i]     = -end(i, 0);
                }
            }

            new_road.rep.points_.push_back(start_point);

            vec3f middle;
            bool okay(projection_intersect(middle,
                                           start_point, start_tan,
                                           end_point,   end_tan));

            if(!okay)
            {
                new_road.rep.points_.push_back(vec3f(start_point + start_tan*4));
                new_road.rep.points_.push_back(vec3f(end_point   + end_tan*4));
            }
            else
                new_road.rep.points_.push_back(middle);

            new_road.rep.points_.push_back(end_point);

            new_road.rep.initialize(0.0f);

            assert(new_road.check());

            assert(!sp.fict_lane);
            const str lane_id(boost::str(boost::format("%s_to_%s_fict_lane") % in->id % out->id));

            lane_map::iterator new_lane_itr(fict_lanes.find(lane_id));
            assert(new_lane_itr == fict_lanes.end());

            new_lane_itr = fict_lanes.insert(new_lane_itr, std::make_pair(lane_id, lane()));
            lane &new_lane = new_lane_itr->second;
            new_lane.id = lane_id;

            {
                lane::road_membership rm;
                rm.parent_road = &new_road;
                rm.lane_position = 0.0f;
                rm.interval[0] = 0.0f;
                rm.interval[1] = 1.0f;
                new_lane.road_memberships.insert(0.0, rm);
            }

            new_lane.start = new hwm::lane::lane_terminus(in);
            new_lane.end   = new hwm::lane::lane_terminus(out);

            new_lane.speedlimit = out->speedlimit;
            new_lane.active     = false;

            in_pair().replace(current, intersection::state::state_pair(sp.in_idx, sp.out_idx, &new_lane));
        }
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

    void intersection::state::activate()
    {
        BOOST_FOREACH(lane_pair &l, fict_lanes)
        {
            l.second.active = true;
        }
    }

    void intersection::state::deactivate()
    {
        BOOST_FOREACH(lane_pair &l, fict_lanes)
        {
            l.second.active = false;
        }
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
        BOOST_FOREACH(state &st, states)
        {
            st.translate(o);
        }

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

    void intersection::build_fictitious_lanes()
    {
        BOOST_FOREACH(state &s, states)
        {
            s.build_fictitious_lanes(*this);
        }
        states[current_state].activate();
    }

    lane *intersection::downstream_lane(const int incoming_ref) const
    {
        assert(current_state < states.size());

        const state::state_pair_in &in_pairs = states[current_state].state_pairs.get<state::in>();
        state::state_pair_in::const_iterator result = in_pairs.find(incoming_ref);

        return (locked || result == in_pairs.end()) ? 0 : result->fict_lane;
    }

    lane *intersection::upstream_lane(const int outgoing_ref) const
    {
        assert(current_state < states.size());

        const state::state_pair_out &out_pairs = states[current_state].state_pairs.get<state::out>();
        state::state_pair_out::const_iterator result = out_pairs.find(outgoing_ref);

        return (locked || result == out_pairs.end()) ? 0 : result->fict_lane;
    }

    void intersection::advance_state()
    {
        states[current_state].deactivate();
        ++current_state;
        if(current_state >= states.size())
            current_state = 0;
        states[current_state].activate();
    }

    void intersection::lock()
    {
        locked = true;
    }
}
