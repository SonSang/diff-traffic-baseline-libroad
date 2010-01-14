#include "hwm_network.hpp"

namespace hwm
{
    network::network(const network &n)
    {
        copy(n);
    }

    void network::copy(const network &n)
    {
        name  = n.name;
        gamma = n.gamma;

        roads         = n.roads;
        lanes         = n.lanes;
        intersections = n.intersections;

        // Fill in pointers in lanes with other lanes
        typedef strhash<lane>::type::value_type lval;
        BOOST_FOREACH(lval &l, lanes)
        {
            const str &lane_id      = l.first;
            lane      &current_lane = l.second;

            // Find lane in 'n' that corresponds to this lane
            const strhash<lane>::type::const_iterator other = n.lanes.find(lane_id);
            assert(other != n.lanes.end());
            assert(other->first == lane_id);

            const lane &other_lane = other->second;

            {
                lane::road_membership::intervals::iterator       my_memb    = current_lane.road_memberships.begin();
                lane::road_membership::intervals::const_iterator other_memb =   other_lane.road_memberships.begin();
                for(; my_memb  != current_lane.road_memberships.end() &&
                    other_memb !=   other_lane.road_memberships.end();
                    ++my_memb, ++other_memb)
                {
                    assert(my_memb->second.parent_road);
                    const strhash<road>::type::iterator my_road = roads.find(other_memb->second.parent_road->id);
                    assert(my_road != roads.end());
                    my_memb->second.parent_road = &(my_road->second);
                }
            }
            {
                lane::adjacency::intervals::iterator       my_adj    = current_lane.left.begin();
                lane::adjacency::intervals::const_iterator other_adj =   other_lane.left.begin();
                for(; my_adj  != current_lane.left.end() &&
                    other_adj !=   other_lane.left.end();
                    ++my_adj, ++other_adj)
                {
                    if(my_adj->second.neighbor)
                    {
                        const strhash<lane>::type::iterator my_lane = lanes.find(other_adj->second.neighbor->id);
                        assert(my_lane != lanes.end());
                        my_adj->second.neighbor = &(my_lane->second);
                    }
                }
            }
            {
                lane::adjacency::intervals::iterator       my_adj    = current_lane.right.begin();
                lane::adjacency::intervals::const_iterator other_adj =   other_lane.right.begin();
                for(; my_adj  != current_lane.right.end() &&
                    other_adj !=   other_lane.right.end();
                    ++my_adj, ++other_adj)
                {
                    if(my_adj->second.neighbor)
                    {
                        const strhash<lane>::type::iterator my_lane = lanes.find(other_adj->second.neighbor->id);
                        assert(my_lane != lanes.end());
                        my_adj->second.neighbor = &(my_lane->second);
                    }
                }
            }
            // if(current_lane.start.inters)
            // {
            //     const strhash<intersection>::type::iterator my_inters = intersections.find(other_lane.start.inters->id);
            //     assert(my_inters != intersections.end());
            //     current_lane.start.inters = &(my_inters->second);
            // }
            // if(current_lane.end.inters)
            // {
            //     const strhash<intersection>::type::iterator my_inters = intersections.find(other_lane.end.inters->id);
            //     assert(my_inters != intersections.end());
            //     current_lane.end.inters = &(my_inters->second);
            // }
        }
        // typedef strhash<intersection>::type::value_type ival;
        // BOOST_FOREACH(ival &i, intersections)
        // {
        //     const str    &intersection_id      = i.first;
        //     intersection &current_intersection = i.second;

        //     const strhash<intersection>::type::const_iterator other = n.intersections.find(i.first);
        //     assert(other != n.intersections.end());
        //     assert(other->first == intersection_id);

        //     const intersection &other_intersection = other->second;

        //     {
        //         std::vector<lane*>::iterator       my_inc     = current_intersection.incoming.begin();
        //         std::vector<lane*>::const_iterator other_inc  =   other_intersection.incoming.begin();
        //         for(; my_inc  != current_intersection.incoming.end() &&
        //             other_inc !=   other_intersection.incoming.end();
        //             ++my_inc, ++other_inc)
        //         {
        //             assert(*my_inc);
        //             const strhash<lane>::type::iterator my_lane = lanes.find((*other_inc)->id);
        //             assert(my_lane != lanes.end());
        //             *my_inc = &(my_lane->second);
        //         }
        //     }

        //     {
        //         std::vector<lane*>::iterator       my_out     = current_intersection.outgoing.begin();
        //         std::vector<lane*>::const_iterator other_out  =   other_intersection.outgoing.begin();
        //         for(; my_out  != current_intersection.outgoing.end() &&
        //             other_out !=   other_intersection.outgoing.end();
        //             ++my_out, ++other_out)
        //         {
        //             assert(*my_out);
        //             const strhash<lane>::type::iterator my_lane = lanes.find((*other_out)->id);
        //             assert(my_lane != lanes.end());
        //             *my_out = &(my_lane->second);
        //         }

        //         std::vector<intersection::state>::iterator       my_state    = current_intersection.states.begin();
        //         std::vector<intersection::state>::const_iterator other_state =   other_intersection.states.begin();
        //         for(;  my_state != current_intersection.states.end() &&
        //             other_state !=   other_intersection.states.end();
        //             ++my_state, ++other_state)
        //         {
        //             *my_state = *other_state;
        //             std::vector<intersection::state::out_id>::iterator       my_out    =    my_state->in_states.begin();
        //             std::vector<intersection::state::out_id>::const_iterator other_out = other_state->in_states.begin();
        //             for(;  my_out !=    my_state->in_states.end() &&
        //                 other_out != other_state->in_states.end();
        //                 ++my_out, ++other_out)
        //             {
        //                 my_out->fict_lane = 0;
        //             }
        //             std::vector<intersection::state::in_id>::iterator       my_in    =    my_state->out_states.begin();
        //             std::vector<intersection::state::in_id>::const_iterator other_in = other_state->out_states.begin();
        //             for(;  my_in !=    my_state->out_states.end() &&
        //                 other_in != other_state->out_states.end();
        //                 ++my_in, ++other_in)
        //             {
        //                 my_in->fict_lane = 0;
        //             }
        //         }
        //}
        //}
    }

    network &network::operator=(const network &n)
    {
        this->network::~network();
        copy(n);
        return *this;
    }

    bool network::check() const
    {
        if(gamma <= 0.0f || gamma >= 1.0f)
            return false;

        if(lane_width <= 0.0f)
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

    void network::translate(const vec3f &o)
    {
        typedef strhash<road>::type::value_type road_pair;
        BOOST_FOREACH(road_pair &rp, roads)
        {
            rp.second.translate(o);
        }
        typedef strhash<intersection>::type::value_type intersection_pair;
        BOOST_FOREACH(intersection_pair &ip, intersections)
        {
            ip.second.translate(o);
        }
    }

    void network::build_intersections()
    {
        typedef strhash<intersection>::type::value_type intersection_pair;
        BOOST_FOREACH(intersection_pair &ip, intersections)
        {
            ip.second.build_shape(lane_width);
        }
    }

    void network::build_fictitious_lanes()
    {
        typedef strhash<intersection>::type::value_type intersection_pair;
        BOOST_FOREACH(intersection_pair &ip, intersections)
        {
            ip.second.build_fictitious_lanes();
        }
    }

    void network::center(const bool z)
    {
        vec3f low(FLT_MAX);
        vec3f high(-FLT_MAX);
        bounding_box(low, high);
        vec3f center((low+high)/2);
        if(!z)
            center[2] = 0;
        translate(vec3f(-center));
    }

    void network::bounding_box(vec3f &low, vec3f &high) const
    {
        typedef strhash<road>::type::value_type rval;
        BOOST_FOREACH(const rval &rv, roads)
        {
            rv.second.bounding_box(low, high);
        }
    }

    typedef strhash<road>::type::iterator         road_itr;
    typedef strhash<lane>::type::iterator         lane_itr;
    typedef strhash<intersection>::type::iterator intersection_itr;

    typedef strhash<sumo::node>::type::value_type      node_pair;
    typedef strhash<sumo::edge_type>::type::value_type type_pair;
    typedef strhash<sumo::edge>::type::value_type      edge_pair;

    template <class T>
    static inline T &retrieve(typename strhash<T>::type &m, const str &id)
    {
        typename strhash<T>::type::iterator entry(m.find(id));
        if(entry == m.end())
        {
            entry = m.insert(entry, std::make_pair(id, T()));
            entry->second.id = id;
        }

        return entry->second;
    }

    network from_sumo(const str &name, const float gamma, const float lane_width, const sumo::network &snet)
    {
        network hnet;
        hnet.name       = name;
        hnet.gamma      = gamma;
        hnet.lane_width = lane_width;

        strhash<size_t>::type node_degree;
        BOOST_FOREACH(const node_pair &np, snet.nodes)
        {
            node_degree.insert(std::make_pair(np.first, 0));
        }

        BOOST_FOREACH(const edge_pair &ep, snet.edges)
        {
            const sumo::edge &e = ep.second;

            ++node_degree[e.from->id];
            ++node_degree[e.to->id];

            road &new_road = retrieve<road>(hnet.roads, e.id);
            new_road.name = new_road.id;

            new_road.rep.points_.reserve(2 + e.shape.size());
            new_road.rep.points_.push_back(vec3f(e.from->xy[0],
                                                 e.from->xy[1],
                                                 0.0f));
            BOOST_FOREACH(const vec2d &v, e.shape)
            {
                new_road.rep.points_.push_back(vec3f(v[0],
                                                     v[1],
                                                     0.0f));
            }
            new_road.rep.points_.push_back(vec3f(e.to->xy[0],
                                                 e.to->xy[1],
                                                 0.0f));
            if(!new_road.rep.initialize())
                throw std::exception();
        }

        BOOST_FOREACH(const strhash<size_t>::type::value_type &ndeg, node_degree)
        {
            if(ndeg.second > 1)
                retrieve<intersection>(hnet.intersections, ndeg.first);
        }

        BOOST_FOREACH(const edge_pair &ep, snet.edges)
        {
            const sumo::edge      &e           = ep.second;
            const sumo::edge_type &et          = *(e.type);
            road                  *parent_road = &retrieve<road>(hnet.roads, e.id);
            std::vector<lane*>     newlanes(et.nolanes);

            intersection *start_inters, *end_inters;
            {
                intersection_itr the_inters = hnet.intersections.find(e.from->id);
                start_inters                = (the_inters == hnet.intersections.end()) ? 0 : &(the_inters->second);
                the_inters                  = hnet.intersections.find(e.to->id);
                end_inters                  = (the_inters == hnet.intersections.end()) ? 0 : &(the_inters->second);
            }

            for(int lanect = 0; lanect < et.nolanes; ++lanect)
            {
                lane &new_lane = retrieve<lane>(hnet.lanes, boost::str(boost::format("%s_%02d") % e.id % lanect));
                newlanes[lanect] = &new_lane;

                new_lane.speedlimit = et.speed;
                lane::road_membership rm;
                rm.parent_road = parent_road;
                rm.interval[0] = 0.0f;
                rm.interval[1] = 1.0f;
                rm.lane_position = lanect;
                new_lane.road_memberships.insert(0.0, rm);

                // new_lane.start.inters = start_inters;
                // if(start_inters)
                // {
                //     start_inters->outgoing.push_back(&new_lane);
                //     new_lane.start.intersect_in_ref = start_inters->outgoing.size()-1;
                // }
                // else
                //     new_lane.start.intersect_in_ref = -1;

                // new_lane.end.inters = end_inters;
                // if(end_inters)
                // {
                //     end_inters->incoming.push_back(&new_lane);
                //     new_lane.end.intersect_in_ref = end_inters->incoming.size()-1;
                // }
                // else
                //     new_lane.end.intersect_in_ref = -1;
            }

            for(int lanect = 0; lanect < et.nolanes; ++lanect)
            {
                if(lanect > 0)
                {
                    lane::adjacency la;
                    la.neighbor = newlanes[lanect-1];
                    la.neighbor_interval[0] = 0.0f;
                    la.neighbor_interval[1] = 1.0f;
                    newlanes[lanect]->left.insert(0.0f, la);
                }
                if(lanect < et.nolanes - 1)
                {
                    lane::adjacency la;
                    la.neighbor = newlanes[lanect+1];
                    la.neighbor_interval[0] = 0.0f;
                    la.neighbor_interval[1] = 1.0f;
                    newlanes[lanect]->right.insert(0.0f, la);
                }
            }
        }

        return hnet;
    }
}
