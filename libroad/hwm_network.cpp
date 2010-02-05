#include "hwm_network.hpp"

namespace hwm
{
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

    void network::translate(const vec3f &o)
    {
        typedef strhash<road>::type::value_type road_pair;
        BOOST_FOREACH(road_pair &rp, roads)
        {
            rp.second.translate(o);
        }
    }

    typedef strhash<road>::type::iterator         road_itr;
    typedef strhash<lane>::type::iterator         lane_itr;
    typedef strhash<intersection>::type::iterator intersection_itr;



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

    network from_sumo(const str &name, const float gamma, const sumo::network &snet)
    {
        typedef strhash<sumo::node>::type::value_type      node_pair;
        typedef strhash<sumo::edge_type>::type::value_type type_pair;
        typedef strhash<sumo::edge>::type::value_type      edge_pair;

        network hnet;
        hnet.name  = name;
        hnet.gamma = gamma;

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

                new_lane.start.inters = start_inters;
                if(start_inters)
                {
                    start_inters->outgoing.push_back(&new_lane);
                    new_lane.start.intersect_in_ref = start_inters->outgoing.size()-1;
                }
                else
                    new_lane.start.intersect_in_ref = -1;

                new_lane.end.inters = end_inters;
                if(end_inters)
                {
                    end_inters->incoming.push_back(&new_lane);
                    new_lane.end.intersect_in_ref = end_inters->incoming.size()-1;
                }
                else
                    new_lane.end.intersect_in_ref = -1;
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

    vec2d bias;
    bool first_for_convert = true;
    const osm::node* last;
    network from_osm(const str &name, const float gamma, osm::network &snet)
    {
        typedef strhash<osm::node>::type::value_type      node_pair;
        typedef strhash<osm::edge_type>::type::value_type type_pair;
        typedef strhash<osm::edge>::type::value_type      edge_pair;
        network hnet;
        hnet.name  = name;
        hnet.gamma = gamma;

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

            if(!new_road.rep.initialize())
                throw std::exception();
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
                intersection_itr the_inters = hnet.intersections.find(e.from);
                start_inters                = (the_inters == hnet.intersections.end()) ? 0 : &(the_inters->second);
                the_inters                  = hnet.intersections.find(e.to);
                end_inters                  = (the_inters == hnet.intersections.end()) ? 0 : &(the_inters->second);
            }

            for(int lanect = 0; lanect < et.nolanes; ++lanect)
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
                rm.lane_position = lanect + 1;
                new_lane.road_memberships.insert(0.0, rm);

                new_lane.start.inters = start_inters;
                if(start_inters)
                {
                    start_inters->outgoing.push_back(&new_lane);
                    new_lane.start.intersect_in_ref = start_inters->outgoing.size()-1;
                }
                else
                    new_lane.start.intersect_in_ref = -1;

                new_lane.end.inters = end_inters;
                if(end_inters)
                {
                    end_inters->incoming.push_back(&new_lane);
                    new_lane.end.intersect_in_ref = end_inters->incoming.size()-1;
                }
                else
                    new_lane.end.intersect_in_ref = -1;

                if (et.oneway == 0)
                {
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
                    rm_rev.lane_position = -1*(lanect + 1);
                    new_reverse_lane.road_memberships.insert(0.0, rm_rev);

                    new_reverse_lane.start.inters = end_inters;
                    if(end_inters)
                    {
                        end_inters->outgoing.push_back(&new_reverse_lane);
                        new_reverse_lane.start.intersect_in_ref = end_inters->outgoing.size()-1;
                    }
                    else
                        new_reverse_lane.start.intersect_in_ref = -1;

                    new_reverse_lane.end.inters = start_inters;
                    if(start_inters)
                    {
                        start_inters->incoming.push_back(&new_reverse_lane);
                        new_reverse_lane.end.intersect_in_ref = start_inters->incoming.size()-1;
                    }
                    else
                        new_reverse_lane.end.intersect_in_ref = -1;
                }

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
        }

        float STATE_DURATION = 20;

        //TODO Use geometric method to get minimal set of traffic states.
        typedef std::pair<const str, osm::intersection> isect_pair;
        BOOST_FOREACH(const isect_pair& i_pair, snet.intersections)
        {


            const osm::intersection& osm_isect = i_pair.second;
            hwm::intersection& hwm_isect = hnet.intersections[osm_isect.id_from_node];

            //Add states for every pairing of roads that are ending here.
            for(int i = 0; i < osm_isect.edges_ending_here.size(); i++)
            {
                osm::edge& i_edge = (*osm_isect.edges_ending_here[i]);
                for(int j = i+1; j < osm_isect.edges_ending_here.size(); j++)
                {
                    osm::edge& j_edge = (*osm_isect.edges_ending_here[j]);
                    hwm::intersection::state state;
                    state.duration = STATE_DURATION;

                    //Create state for two roads.
                    int max_lanes = std::max(hwm_isect.incoming.size(), hwm_isect.outgoing.size());

                    for (int k = 0; k < max_lanes; k++)
                    {
                        //Create an in_id and and out_id
                        hwm::intersection::state::in_id in;
                        in.in_ref = -1;
                        in.fict_lane = NULL;

                        state.out_states.push_back(in);

                        hwm::intersection::state::out_id out;
                        out.out_ref = -1;
                        out.fict_lane = NULL;

                        state.in_states.push_back(out);
                    }


                    //make every incoming lane (in_id)th element of in_states an out_id with out_ref of matching outgoing lane

                    //I need the incoming id for each lane

                    //These are "incoming" roads, so forward lanes will be incoming lanes, and reverse lanes will be outgoing lanes.
                    //And the lane "end" is this intersection

                    //make every outgoing lane (out_id)th element of out_states an in_id with in_ref of matching incoming lane

                    for (int k = 0; k < roads_to_lanes[i_edge.id].first.size(); k++)
                    {
                        lane* l = roads_to_lanes[i_edge.id].first[k];

                        //Other incoming road, so we need its reverse lanes
                        if (k < roads_to_lanes[j_edge.id].second.size())
                        {
                            lane* l_j = roads_to_lanes[j_edge.id].second[k];
                            state.in_states[l->end.intersect_in_ref].out_ref = l_j->start.intersect_in_ref;
                            state.out_states[l_j->start.intersect_in_ref].in_ref = l->end.intersect_in_ref;

                        }
                    }

                    //Now match up the forward lanes of the j_edge with the reverse of the i_edge
                    for (int k = 0; k < roads_to_lanes[j_edge.id].first.size(); k++)
                    {
                        lane* l = roads_to_lanes[j_edge.id].first[k];

                        if (k < roads_to_lanes[i_edge.id].second.size())
                        {
                            lane* l_i = roads_to_lanes[i_edge.id].second[k];
                            state.in_states[l->end.intersect_in_ref].out_ref = l_i->start.intersect_in_ref;
                            state.out_states[l_i->start.intersect_in_ref].in_ref = l->end.intersect_in_ref;
                        }
                    }


                    hwm_isect.states.push_back(state);

                }
            }



            //Add states for every pairing of roads that are starting here.
            for(int i = 0; i < osm_isect.edges_starting_here.size(); i++)
            {
                osm::edge& i_edge = (*osm_isect.edges_starting_here[i]);
                for(int j = i+1; j < osm_isect.edges_starting_here.size(); j++)
                {
                    osm::edge& j_edge = (*osm_isect.edges_starting_here[j]);
                    hwm::intersection::state state;
                    state.duration = STATE_DURATION;

                    //Create state for two roads.
                    int max_lanes = std::max(hwm_isect.incoming.size(), hwm_isect.outgoing.size());

                    for (int k = 0; k < max_lanes; k++)
                    {
                        //Create an in_id and and out_id
                        hwm::intersection::state::in_id in;
                        in.in_ref = -1;
                        in.fict_lane = NULL;

                        state.out_states.push_back(in);

                        hwm::intersection::state::out_id out;
                        out.out_ref = -1;
                        out.fict_lane = NULL;

                        state.in_states.push_back(out);
                    }

                    //make every incoming lane (in_id)th element of in_states an out_id with out_ref of matching outgoing lane

                    //These are "outgoing" roads, so reverse lanes will be incoming lanes, and forward lanes will be outgoing lanes.
                    //And the lane "start" is this intersection

                    for (int k = 0; k < roads_to_lanes[i_edge.id].second.size(); k++)
                    {
                        lane* l = roads_to_lanes[i_edge.id].second[k];

                        //Other outgoing road, so we need its forward lanes
                        if (k < roads_to_lanes[j_edge.id].first.size())
                        {
                            lane* l_j = roads_to_lanes[j_edge.id].first[k];
                            state.in_states[l->end.intersect_in_ref].out_ref = l_j->start.intersect_in_ref;
                            state.out_states[l_j->start.intersect_in_ref].in_ref = l->end.intersect_in_ref;
                        }
                    }

                    //make every outgoing lane (out_id)th element of out_states an in_id with in_ref of matching incoming lane
                    for (int k = 0; k < roads_to_lanes[j_edge.id].second.size(); k++)
                    {
                        lane* l = roads_to_lanes[j_edge.id].second[k];

                        if (k < roads_to_lanes[i_edge.id].first.size())
                        {
                            lane* l_i = roads_to_lanes[i_edge.id].first[k];
                            state.in_states[l->end.intersect_in_ref].out_ref = l_i->start.intersect_in_ref;
                            state.out_states[l_i->start.intersect_in_ref].in_ref = l->end.intersect_in_ref;
                        }
                    }
                    hwm_isect.states.push_back(state);
                }
            }


            //Every pair of outgoing to incoming roads
            for(int i = 0; i < osm_isect.edges_starting_here.size(); i++)
            {
                osm::edge& i_edge = (*osm_isect.edges_starting_here[i]);
                for(int j = 0; j < osm_isect.edges_ending_here.size(); j++)
                {
                    osm::edge& j_edge = (*osm_isect.edges_ending_here[j]);
                    hwm::intersection::state state;
                    state.duration = STATE_DURATION;

                    //Create state for two roads.
                    int max_lanes = std::max(hwm_isect.incoming.size(), hwm_isect.outgoing.size());

                    for (int k = 0; k < max_lanes; k++)
                    {
                        //Create an in_id and and out_id
                        hwm::intersection::state::in_id in;
                        in.in_ref = -1;
                        in.fict_lane = NULL;

                        state.out_states.push_back(in);

                        hwm::intersection::state::out_id out;
                        out.out_ref = -1;
                        out.fict_lane = NULL;

                        state.in_states.push_back(out);
                    }

                    //Match the reverse lanes (incoming) of the road starting here
                    // with the reverse lanes (outgoing) of the road ending here

                    for (int k = 0; k < roads_to_lanes[i_edge.id].second.size(); k++)
                    {
                        lane* l = roads_to_lanes[i_edge.id].second[k];

                        //Other incoming road, so we need its reverse lanes
                        if (k < roads_to_lanes[j_edge.id].second.size())
                        {
                            lane* l_j = roads_to_lanes[j_edge.id].second[k];
                            state.in_states[l->end.intersect_in_ref].out_ref = l_j->start.intersect_in_ref;

                            state.out_states[l_j->start.intersect_in_ref].in_ref = l->end.intersect_in_ref;

                        }
                    }

                    //Match the forward lanes (outgoing) of the road ending here
                    // with the forward lanes (incoming) of the road starting here.

                    for (int k = 0; k < roads_to_lanes[j_edge.id].first.size(); k++)
                    {
                        lane* l = roads_to_lanes[j_edge.id].first[k];

                        if (k < roads_to_lanes[i_edge.id].first.size())
                        {
                            lane* l_i = roads_to_lanes[i_edge.id].first[k];
                            state.in_states[l->end.intersect_in_ref].out_ref = l_i->end.intersect_in_ref;

                            state.out_states[l_i->start.intersect_in_ref].in_ref = l->end.intersect_in_ref;

                        }
                    }

                    hwm_isect.states.push_back(state);
                }
            }

        }

        return hnet;
    }
}
