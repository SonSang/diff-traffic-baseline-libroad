#ifndef _SUMO_NETWORK_HPP_
#define _SUMO_NETWORK_HPP_

#include "libroad_common.hpp"

namespace sumo
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
        double length;
    };

    struct edge
    {
        struct shape_t : public std::vector<vec2d>
        {
        };
        typedef enum {center, right} SPREAD;

        str        id;
        node*      from;
        node*      to;
        edge_type* type;
        shape_t    shape;
        SPREAD     spread;
    };

    struct network
    {
        network() : anon_node_count(0), anon_edge_type_count(0)
        {}

        strhash<node>::type      nodes;
        size_t                   anon_node_count;
        strhash<edge_type>::type types;
        size_t                   anon_edge_type_count;
        strhash<edge>::type      edges;

        bool check_edge(const edge &e) const;
        bool check_node(const node &n) const;
        bool check() const;
    };

    network load_xml_network(const char *node_file,
                             const char *edge_type_file,
                             const char *edge_file);
}

inline std::ostream &operator<<(std::ostream &o, const sumo::node::TYPES &t)
{
    switch(t)
    {
    case sumo::node::priority:
        o << "priority";
        break;
    case sumo::node::traffic_light:
        o << "traffic_light";
        break;
    default:
        o << "unknown_node_type";
        break;
    }
    return o;
}

inline std::istream &operator>>(std::istream &i, sumo::node::TYPES &t)
{
    str src;
    i >> src;

    if(src == "priority")
        t = sumo::node::priority;
    else if(src == "traffic_light")
        t = sumo::node::traffic_light;
    else
        throw std::exception();

    return i;
}

inline std::ostream &operator<<(std::ostream &o, const sumo::edge::SPREAD &t)
{
    switch(t)
    {
    case sumo::edge::center:
        o << "center";
        break;
    case sumo::edge::right:
        o << "right";
        break;
    default:
        o << "unknown_spread_type";
        break;
    }
    return o;
}

inline std::istream &operator>>(std::istream &i, sumo::edge::SPREAD &t)
{
    str src;
    i >> src;

    if(src == "center")
        t = sumo::edge::center;
    else
        t = sumo::edge::right;

    return i;
}


#endif
