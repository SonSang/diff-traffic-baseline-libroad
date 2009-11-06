#ifndef _OSM_NETWORK_HPP_
#define _OSM_NETWORK_HPP_

#include "libroad_common.hpp"

namespace osm
{
    struct node
    {
        typedef enum {priority, traffic_light, unknown} TYPES;

        str   id;
        vec2d xy;
        TYPES type;
    };

    struct edge_type
    {
        str    id;
        int    nolanes;
        double speed;
        int    priority;
        bool   oneway;

    };

    struct shape_t : public std::vector<node>
    {
    };

    struct edge
    {

        typedef enum {center, right} SPREAD;

        str        id;
        str      from;
        str      to;
        edge_type* type;
        shape_t    shape;
        SPREAD     spread;
        str        highway_class;
    };

    struct network
    {
        network()
        {}

        strhash<node>::type      nodes;
        strhash<edge_type>::type types;
        strhash<edge>::type      edges;

        strhash<edge>::type      road_segs;

        strhash<int>::type      node_degrees;

        bool check_edge(const edge &e) const;
        bool check_node(const node &n) const;
        bool check() const;
        bool compute_edge_types();
        bool draw_network();
        //        bool compute_intersections();
        bool compute_node_degrees();
        bool join_logical_roads();
        bool split_into_road_segments();
        bool join(edge* , edge*);
        bool check_nodes();
    };

    network load_xml_network(const char *osm_file);

}

inline std::ostream &operator<<(std::ostream &o, const osm::node::TYPES &t)
{
    switch(t)
    {
    case osm::node::priority:
        o << "priority";
        break;
    case osm::node::traffic_light:
        o << "traffic_light";
        break;
    default:
        o << "unknown_node_type";
        break;
    }
    return o;
}

inline std::istream &operator>>(std::istream &i, osm::node::TYPES &t)
{
    str src;
    i >> src;

    if(src == "priority")
        t = osm::node::priority;
    else if(src == "traffic_light")
        t = osm::node::traffic_light;
    else
        throw std::exception();

    return i;
}

inline std::ostream &operator<<(std::ostream &o, const osm::edge::SPREAD &t)
{
    switch(t)
    {
    case osm::edge::center:
        o << "center";
        break;
    case osm::edge::right:
        o << "right";
        break;
    default:
        o << "unknown_spread_type";
        break;
    }
    return o;
}

inline std::istream &operator>>(std::istream &i, osm::edge::SPREAD &t)
{
    str src;
    i >> src;

    if(src == "center")
        t = osm::edge::center;
    else
        t = osm::edge::right;

    return i;
}

    template <class T>
    static inline T* retrieve(typename strhash<T>::type &m, const str &id)
    {
        typedef typename strhash<T>::type val;
        typename strhash<T>::type::iterator entry(m.find(id));
        if(entry == m.end())
            m.insert(entry, std::make_pair(id, T()));

        return &(m[id]);
    }


#endif
