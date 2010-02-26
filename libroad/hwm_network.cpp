#include "hwm_network.hpp"

namespace hwm
{
    float maximum_cornering_speed(const float radius, const float g, const float static_friction)
    {
        return std::sqrt(radius*g*static_friction);
    }

    network::network(const network &n)
    {
        copy(n);
    }

    void network::copy(const network &n)
    {
        name       = n.name;
        gamma      = n.gamma;
        lane_width = n.lane_width;

        roads         = n.roads;
        lanes         = n.lanes;
        intersections = n.intersections;

        // Fill in pointers in lanes with other lanes
        BOOST_FOREACH(lane_pair &l, lanes)
        {
            const str &lane_id      = l.first;
            lane      &current_lane = l.second;

            // Find lane in 'n' that corresponds to this lane
            const lane_map::const_iterator other = n.lanes.find(lane_id);
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
                    const road_map::iterator my_road = roads.find(other_memb->second.parent_road->id);
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
                        const lane_map::iterator my_lane = lanes.find(other_adj->second.neighbor->id);
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
                        const lane_map::iterator my_lane = lanes.find(other_adj->second.neighbor->id);
                        assert(my_lane != lanes.end());
                        my_adj->second.neighbor = &(my_lane->second);
                    }
                }
            }

            current_lane.start->update_pointers(*this);
            current_lane.end->update_pointers(*this);
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

    void network::check() const
    {
        if(gamma <= 0.0f || gamma >= 1.0f)
            throw std::runtime_error("Gamma in network out of range");

        if(lane_width <= 0.0f)
            throw std::runtime_error("lane_width in network out of range");

        BOOST_FOREACH(const road_pair &r, roads)
        {
            if(r.first != r.second.id)
                throw std::runtime_error("Container id and local id mismatch for road");
            r.second.check();
        }

        BOOST_FOREACH(const lane_pair &l, lanes)
        {
            if(l.first != l.second.id)
                throw std::runtime_error("Container id and local id mismatch for lane");
            l.second.check();
        }

        BOOST_FOREACH(const intersection_pair &i, intersections)
        {
            if(i.first != i.second.id)
                throw std::runtime_error("Container id and local id mismatch for road");
               i.second.check();
        }
    }

    void network::translate(const vec3f &o)
    {
        BOOST_FOREACH(road_pair &rp, roads)
        {
            rp.second.translate(o);
        }

        BOOST_FOREACH(intersection_pair &ip, intersections)
        {
            ip.second.translate(o);
        }
    }

    void network::build_intersections()
    {
        BOOST_FOREACH(intersection_pair &ip, intersections)
        {
            ip.second.build_shape(lane_width);
        }
    }

    void network::build_fictitious_lanes()
    {
        BOOST_FOREACH(intersection_pair &ip, intersections)
        {
            ip.second.build_fictitious_lanes();
        }
    }

    void network::auto_scale_memberships()
    {
        BOOST_FOREACH(lane_pair &lp, lanes)
        {
            lp.second.auto_scale_memberships();
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
        BOOST_FOREACH(const road_pair &rv, roads)
        {
            rv.second.bounding_box(low, high);
        }
    }

    template <class T>
    static inline T &retrieve(typename strhash<T>::type &m, const str &id)
    {
        assert(id != str());
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
        typedef strhash<sumo::node>::type::value_type      sumo_node_pair;
        typedef strhash<sumo::edge_type>::type::value_type sumo_type_pair;
        typedef strhash<sumo::edge>::type::value_type      sumo_edge_pair;

        network hnet;
        hnet.name       = name;
        hnet.gamma      = gamma;
        hnet.lane_width = lane_width;

        strhash<size_t>::type node_degree;
        BOOST_FOREACH(const sumo_node_pair &np, snet.nodes)
        {
            node_degree.insert(std::make_pair(np.first, 0));
        }

        BOOST_FOREACH(const sumo_edge_pair &ep, snet.edges)
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
            if(!new_road.rep.initialize_from_polyline(lane_width, new_road.rep.points_))
                throw std::runtime_error("Failed to initialize arc_road in from_sumo");
        }

        BOOST_FOREACH(const strhash<size_t>::type::value_type &ndeg, node_degree)
        {
            if(ndeg.second > 1)
                retrieve<intersection>(hnet.intersections, ndeg.first);
        }

        BOOST_FOREACH(const sumo_edge_pair &ep, snet.edges)
        {
            const sumo::edge      &e           = ep.second;
            const sumo::edge_type &et          = *(e.type);
            road                  *parent_road = &retrieve<road>(hnet.roads, e.id);
            std::vector<lane*>     newlanes(et.nolanes);

            intersection *start_inters, *end_inters;
            {
                intersection_map::iterator the_inters = hnet.intersections.find(e.from->id);
                start_inters                          = (the_inters == hnet.intersections.end()) ? 0 : &(the_inters->second);
                the_inters                            = hnet.intersections.find(e.to->id);
                end_inters                            = (the_inters == hnet.intersections.end()) ? 0 : &(the_inters->second);
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

    network from_osm(const str &name, const float gamma, const float lane_width, osm::network &snet)
    {
        typedef strhash<osm::node>::type::value_type      node_pair;
        typedef strhash<osm::edge_type>::type::value_type type_pair;
        typedef strhash<osm::edge>::type::value_type      edge_pair;
        network                                           hnet;
        hnet.name       = name;
        hnet.gamma      = gamma;
        hnet.lane_width = lane_width;

        strhash<size_t>::type node_degree;
        BOOST_FOREACH(const node_pair &np, snet.nodes)
        {
            node_degree.insert(std::make_pair(np.first, 0));
        }

        BOOST_FOREACH(const osm::edge& e, snet.edges)
        {
            ++node_degree[e.shape[0]->id];
            ++node_degree[e.shape.back()->id];

            road &new_road = retrieve<road>(hnet.roads, e.id);
            new_road.name = new_road.id;

            new_road.rep.points_.reserve(2 + e.shape.size());

            const osm::node* last;
            BOOST_FOREACH(const osm::node *n, e.shape)
            {
                if (n->id != e.shape[0]->id)
                {
                    if(sqrt(pow(n->xy[0] - last->xy[0],2)
                            + pow(n->xy[1] - last->xy[1],2)) > 1e-7){
                        new_road.rep.points_.push_back(vec3f(n->xy[0],
                                                             n->xy[1],
                                                             n->xy[2]));
                    }
                }
                else{
                    new_road.rep.points_.push_back(vec3f(n->xy[0],n->xy[1],n->xy[2]));
                }
                last = n;
            }

            if(!new_road.rep.initialize_from_polyline(0.7f, new_road.rep.points_))
                throw std::runtime_error("Failed to initialize arc_road in from_osm");
        }

        BOOST_FOREACH(const strhash<size_t>::type::value_type &ndeg, node_degree)
        {
            if(ndeg.second > 1){
                assert(ndeg.first != "");
                retrieve<intersection>(hnet.intersections, ndeg.first);
            }
        }

        typedef std::pair<std::vector<lane*>, std::vector<lane*> > in_and_out;
        std::map<str, in_and_out> roads_to_lanes;

        BOOST_FOREACH(const osm::edge& e, snet.edges)
        {
            const osm::edge_type &et          = *(e.type);
            road                  *parent_road = &retrieve<road>(hnet.roads, e.id);

            std::vector<lane*>     newlanes(et.nolanes);
            std::vector<lane*>     new_reverse_lanes(et.nolanes);

            intersection *start_inters, *end_inters;
            {
                intersection_map::iterator the_inters = hnet.intersections.find(e.from);
                start_inters                          = (the_inters == hnet.intersections.end()) ? 0 : &(the_inters->second);
                the_inters                            = hnet.intersections.find(e.to);
                end_inters                            = (the_inters == hnet.intersections.end()) ? 0 : &(the_inters->second);
            }

            for(int lanect = 0; lanect < et.nolanes; ++lanect)
            {
                if (et.oneway == 0)
                {
                    lane &new_lane = retrieve<lane>(hnet.lanes, boost::str(boost::format("%s_%02d") % e.id % lanect));
                    newlanes[lanect] = &new_lane;

                    //Store road to lane pointers.
                    roads_to_lanes[e.id].first.push_back(&new_lane);

                    new_lane.speedlimit = et.speed;
                    lane::road_membership rm;
                    rm.parent_road = parent_road;
                    rm.interval[0] = 0.0f;
                    rm.interval[1] = 1.0f;
                    rm.lane_position = (2.5)*(-0.5 + -1*(lanect));

                    new_lane.road_memberships.insert(0.0, rm);

                    //Create intersections only if one has not been assigned.
                    if (!new_lane.start)
                    {
                        if(start_inters)
                        {
                            start_inters->outgoing.push_back(&new_lane);
                            new_lane.start = new lane::intersection_terminus(start_inters, start_inters->outgoing.size()-1);
                        }
                        else
                            new_lane.start = new lane::terminus();
                    }
                    if (!new_lane.end)
                    {
                        if(end_inters)
                        {
                            end_inters->incoming.push_back(&new_lane);
                            new_lane.end = new lane::intersection_terminus(end_inters, end_inters->incoming.size()-1);
                        }
                        else
                            new_lane.end = new lane::terminus();
                    }

                    //Add reverse lanes  (TODO one way roads)
                    lane &new_reverse_lane = retrieve<lane>(hnet.lanes, boost::str(boost::format("%s_%02d_reverse") % e.id % lanect));
                    new_reverse_lanes[lanect] = &new_reverse_lane;

                    //Stare road to lane pointers
                    roads_to_lanes[e.id].second.push_back(&new_reverse_lane);

                    new_reverse_lane.speedlimit = et.speed;
                    lane::road_membership rm_rev;
                    rm_rev.parent_road = parent_road;
                    rm_rev.interval[0] = 1.0f;
                    rm_rev.interval[1] = 0.0f;
                    rm_rev.lane_position = (2.5)*(0.5 + (lanect));;

                    new_reverse_lane.road_memberships.insert(0.0, rm_rev);
                    //Create intersections only if one has not been assigned.
                    if (!new_reverse_lane.start)
                    {
                        if(end_inters)
                        {
                            end_inters->outgoing.push_back(&new_reverse_lane);
                            new_reverse_lane.start = new lane::intersection_terminus(end_inters, end_inters->outgoing.size()-1);
                        }
                        else
                            new_reverse_lane.start = new lane::terminus();
                    }

                    if (!new_reverse_lane.end)
                    {
                        if(start_inters)
                        {
                            start_inters->incoming.push_back(&new_reverse_lane);
                            new_reverse_lane.end = new lane::intersection_terminus(start_inters, start_inters->incoming.size()-1);
                        }
                        else
                            new_reverse_lane.end = new lane::terminus();
                    }
                }
                else //Oneway road
                {
                    lane &new_lane = retrieve<lane>(hnet.lanes, boost::str(boost::format("%s_%02d") % e.id % lanect));
                    newlanes[lanect] = &new_lane;

                    //Store road to lane pointers.
                    roads_to_lanes[e.id].first.push_back(&new_lane);

                    new_lane.speedlimit = et.speed;
                    lane::road_membership rm;
                    rm.parent_road = parent_road;
                    rm.interval[0] = 0.0f;
                    rm.interval[1] = 1.0f;

                    //TODO use lane width value, not constant

                    float position = (2.5)*(lanect + -((et.nolanes - 1)/2.0));
                    //Units possibly in half lane widths...

                    rm.lane_position = position;
                    new_lane.road_memberships.insert(0.0, rm);

                    //Create intersections only if one has not been assigned.
                    if (!new_lane.start)
                    {
                        if(start_inters)
                        {
                            start_inters->outgoing.push_back(&new_lane);
                            new_lane.start = new lane::intersection_terminus(start_inters, start_inters->outgoing.size()-1);
                        }
                        else
                            new_lane.start = new lane::terminus();
                    }

                    if (!new_lane.end)
                    {
                        if(end_inters)
                        {
                            end_inters->incoming.push_back(&new_lane);
                            new_lane.end = new lane::intersection_terminus(end_inters, end_inters->incoming.size()-1);
                        }
                        else
                            new_lane.end = new lane::terminus();
                    }
                }
            }

            //Add merging lanes TODO
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

                if (et.oneway == 0)
                {
                    //Repeat for reverse lanes
                    if(lanect > 0)
                    {
                        lane::adjacency la;
                        la.neighbor = new_reverse_lanes[lanect-1];
                        la.neighbor_interval[0] = 0.0f;
                        la.neighbor_interval[1] = 1.0f;
                        new_reverse_lanes[lanect]->left.insert(0.0f, la);
                    }
                    if(lanect < et.nolanes - 1)
                    {
                        lane::adjacency la;
                        la.neighbor = new_reverse_lanes[lanect+1];
                        la.neighbor_interval[0] = 0.0f;
                        la.neighbor_interval[1] = 1.0f;
                        new_reverse_lanes[lanect]->right.insert(0.0f, la);
                    }
                }
            }

            BOOST_FOREACH(const osm::edge::lane& l, e.additional_lanes)
            {
                str id = boost::str(boost::format("%1%_%2%_%3%_%4%") % e.id % l.start_t % l.end_t % l.offset);
                lane &new_lane = retrieve<lane>(hnet.lanes, id);
                //Store road to lane pointers.
                roads_to_lanes[e.id].first.push_back(&new_lane);

                new_lane.speedlimit = et.speed;
                lane::road_membership rm;
                rm.parent_road = parent_road;
                rm.interval[0] = l.start_t;
                rm.interval[1] = l.end_t;

                //TODO use lane width value, not constant

                float position = l.offset;
                //Units possibly in half lane widths...

                rm.lane_position = position;
                new_lane.road_memberships.insert(0.0, rm);

                lane &ramp_lane = retrieve<lane>(hnet.lanes, boost::str(boost::format("%s_%02d") % l.ramp_id % 0));

                //Create lane terminus so that ramp flows into extra lane.
                if (l.offramp)
                {
                    lane::lane_terminus* foo = new lane::lane_terminus();
                    foo->adjacent_lane = &ramp_lane;
                    new_lane.end = foo;

                    delete ramp_lane.start;
                    lane::lane_terminus* bar = new lane::lane_terminus();
                    bar->adjacent_lane = &new_lane;
                    ramp_lane.start = bar;


                    new_lane.start = new lane::terminus();

                }
                else
                {
                    lane::lane_terminus* foo = new lane::lane_terminus();
                    foo->adjacent_lane = &ramp_lane;
                    new_lane.start = foo;

                    delete ramp_lane.end;
                    lane::lane_terminus* bar = new lane::lane_terminus();
                    bar->adjacent_lane = &new_lane;
                    ramp_lane.end = bar;

                    new_lane.end = new lane::terminus();
                }

                lane::adjacency la;
                la.neighbor = newlanes[0];
                la.neighbor_interval[0] = l.start_t;
                la.neighbor_interval[1] = l.end_t;
                new_lane.left.insert(0.0f, la);
            }
        }

        typedef strhash<hwm::lane>::type::value_type hwm_l_pair;
        BOOST_FOREACH(const hwm_l_pair& l, hnet.lanes)
        {
            l.second.check();
        }

        float STATE_DURATION = 20;

        //TODO Use geometric method to get minimal set of traffic states.

        typedef std::pair<const str, osm::intersection> isect_pair;
        BOOST_FOREACH(const isect_pair& i_pair, snet.intersections)
        {
            const osm::intersection& osm_isect = i_pair.second;

            assert(osm_isect.id_from_node != "");

            hwm::intersection& hwm_isect = hnet.intersections[osm_isect.id_from_node];
            if (hwm_isect.id == "")
            {
                assert(0);
            }

            //Add states for every pairing of roads that are ending here.
            for(int i = 0; i < static_cast<int>(osm_isect.edges_ending_here.size()); i++)
            {
                osm::edge& i_edge = (*osm_isect.edges_ending_here[i]);
                for(int j = i+1; j < static_cast<int>(osm_isect.edges_ending_here.size()); j++)
                {
                    osm::edge& j_edge = (*osm_isect.edges_ending_here[j]);
                    hwm::intersection::state state;
                    state.duration = STATE_DURATION;

                    //make every incoming lane (in_id)th element of in_states an out_id with out_ref of matching outgoing lane

                    //I need the incoming id for each lane

                    //These are "incoming" roads, so forward lanes will be incoming lanes, and reverse lanes will be outgoing lanes.
                    //And the lane "end" is this intersection

                    //make every outgoing lane (out_id)th element of out_states an in_id with in_ref of matching incoming lane

                    for (int k = 0; k < static_cast<int>(roads_to_lanes[i_edge.id].first.size()); k++)
                    {
                        lane* l = roads_to_lanes[i_edge.id].first[k];
                        const lane::intersection_terminus *l_it = dynamic_cast<lane::intersection_terminus*>(l->end);
                        assert(l_it);

                        //Other incoming road, so we need its reverse lanes
                        if (k < static_cast<int>(roads_to_lanes[j_edge.id].second.size()))
                        {
                            lane* l_j = roads_to_lanes[j_edge.id].second[k];
                            const lane::intersection_terminus *l_j_it = dynamic_cast<lane::intersection_terminus*>(l_j->start);
                            assert(l_j_it);
                            state.state_pairs.insert(intersection::state::state_pair(l_it->intersect_in_ref, l_j_it->intersect_in_ref));
                        }
                    }

                    //Now match up the forward lanes of the j_edge with the reverse of the i_edge
                    for (int k = 0; k < static_cast<int>(roads_to_lanes[j_edge.id].first.size()); k++)
                    {
                        lane* l = roads_to_lanes[j_edge.id].first[k];
                        const lane::intersection_terminus *l_it = dynamic_cast<lane::intersection_terminus*>(l->end);
                        assert(l_it);

                        if (k < static_cast<int>(roads_to_lanes[i_edge.id].second.size()))
                        {
                            lane* l_i = roads_to_lanes[i_edge.id].second[k];
                            const lane::intersection_terminus *l_i_it = dynamic_cast<lane::intersection_terminus*>(l_i->start);
                            assert(l_i_it);

                            state.state_pairs.insert(intersection::state::state_pair(l_it->intersect_in_ref, l_i_it->intersect_in_ref));
                        }
                    }

                    hwm_isect.states.push_back(state);
                }
            }

            typedef strhash<hwm::intersection>::type::value_type hwm_i_pair;
            BOOST_FOREACH(const hwm_i_pair& i, hnet.intersections)
            {
                assert(i.second.id != "");
            }

            //Add states for every pairing of roads that are starting here.
            for(int i = 0; i < static_cast<int>(osm_isect.edges_starting_here.size()); i++)
            {
                osm::edge& i_edge = (*osm_isect.edges_starting_here[i]);
                for(int j = i+1; j < static_cast<int>(osm_isect.edges_starting_here.size()); j++)
                {
                    osm::edge& j_edge = (*osm_isect.edges_starting_here[j]);
                    hwm::intersection::state state;
                    state.duration = STATE_DURATION;

                    //make every incoming lane (in_id)th element of in_states an out_id with out_ref of matching outgoing lane

                    //These are "outgoing" roads, so reverse lanes will be incoming lanes, and forward lanes will be outgoing lanes.
                    //And the lane "start" is this intersection

                    for (int k = 0; k < static_cast<int>(roads_to_lanes[i_edge.id].second.size()); k++)
                    {
                        lane* l = roads_to_lanes[i_edge.id].second[k];
                        const lane::intersection_terminus *l_it = dynamic_cast<lane::intersection_terminus*>(l->end);
                        assert(l_it);

                        //Other outgoing road, so we need its forward lanes
                        if (k < static_cast<int>(roads_to_lanes[j_edge.id].first.size()))
                        {
                            lane* l_j = roads_to_lanes[j_edge.id].first[k];
                            const lane::intersection_terminus *l_j_it = dynamic_cast<lane::intersection_terminus*>(l_j->start);
                            assert(l_j_it);
                            state.state_pairs.insert(intersection::state::state_pair(l_it->intersect_in_ref, l_j_it->intersect_in_ref));
                        }
                    }

                    //make every outgoing lane (out_id)th element of out_states an in_id with in_ref of matching incoming lane
                    for (int k = 0; k < static_cast<int>(roads_to_lanes[j_edge.id].second.size()); k++)
                    {
                        lane* l = roads_to_lanes[j_edge.id].second[k];
                        const lane::intersection_terminus *l_it = dynamic_cast<lane::intersection_terminus*>(l->end);
                        assert(l_it);

                        if (k < static_cast<int>(roads_to_lanes[i_edge.id].first.size()))
                        {
                            lane* l_i = roads_to_lanes[i_edge.id].first[k];
                            const lane::intersection_terminus *l_i_it = dynamic_cast<lane::intersection_terminus*>(l_i->start);
                            assert(l_i_it);
                            state.state_pairs.insert(intersection::state::state_pair(l_it->intersect_in_ref, l_i_it->intersect_in_ref));
                        }
                    }
                    hwm_isect.states.push_back(state);
                }
            }

            //Every pair of outgoing to incoming roads
            for(int i = 0; i < static_cast<int>(osm_isect.edges_starting_here.size()); i++)
            {
                osm::edge& i_edge = (*osm_isect.edges_starting_here[i]);
                for(int j = 0; j < static_cast<int>(osm_isect.edges_ending_here.size()); j++)
                {
                    osm::edge& j_edge = (*osm_isect.edges_ending_here[j]);
                    hwm::intersection::state state;
                    state.duration = STATE_DURATION;

                    //Match the reverse lanes (incoming) of the road starting here
                    // with the reverse lanes (outgoing) of the road ending here

                    for (int k = 0; k < static_cast<int>(roads_to_lanes[i_edge.id].second.size()); k++)
                    {
                        lane* l = roads_to_lanes[i_edge.id].second[k];
                        const lane::intersection_terminus *l_it = dynamic_cast<lane::intersection_terminus*>(l->end);
                        assert(l_it);

                        //Other incoming road, so we need its reverse lanes
                        if (k < static_cast<int>(roads_to_lanes[j_edge.id].second.size()))
                        {
                            lane* l_j = roads_to_lanes[j_edge.id].second[k];
                            const lane::intersection_terminus *l_j_it = dynamic_cast<lane::intersection_terminus*>(l_j->start);
                            assert(l_j_it);
                            state.state_pairs.insert(intersection::state::state_pair(l_it->intersect_in_ref, l_j_it->intersect_in_ref));
                        }
                    }

                    //Match the forward lanes (outgoing) of the road ending here
                    // with the forward lanes (incoming) of the road starting here.

                    for (int k = 0; k < static_cast<int>(roads_to_lanes[j_edge.id].first.size()); k++)
                    {
                        lane* l = roads_to_lanes[j_edge.id].first[k];
                        const lane::intersection_terminus *l_it = dynamic_cast<lane::intersection_terminus*>(l->end);
                        assert(l_it);

                        if (k < static_cast<int>(roads_to_lanes[i_edge.id].first.size()))
                        {
                            lane* l_i = roads_to_lanes[i_edge.id].first[k];
                            const lane::intersection_terminus *l_i_it = dynamic_cast<lane::intersection_terminus*>(l_i->start);
                            assert(l_i_it);

                            state.state_pairs.insert(intersection::state::state_pair(l_it->intersect_in_ref, l_i_it->intersect_in_ref));
                        }
                    }

                    hwm_isect.states.push_back(state);
                }
            }
        }

        typedef strhash<hwm::intersection>::type::value_type hwm_i_pair;
        BOOST_FOREACH(const hwm_i_pair& i, hnet.intersections)
        {
            assert(i.second.id != "");
        }

        return hnet;
    }
}
