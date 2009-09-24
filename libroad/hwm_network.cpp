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

    network from_sumo(const str &name, const float gamma, const sumo::network &snet)
    {
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
}
