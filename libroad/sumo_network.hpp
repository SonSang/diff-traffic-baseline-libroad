#ifndef _SUMO_NETWORK_HPP_
#define _SUMO_NETWORK_HPP_

#include <vector>
#include <map>
#include <tvmet/Vector.h>
#include <glibmm/ustring.h>
#include <libxml++/libxml++.h>
#include <libxml++/parsers/textreader.h>

#include "xml_util.hpp"

typedef tvmet::Vector<double, 2> vec2d;

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

        std::map<const str, node>      nodes;
        size_t                         anon_node_count;

        std::map<const str, edge_type> types;
        size_t                         anon_edge_type_count;

        std::map<const str, edge>      edges;

        node      *anon_node(vec2d &pos);
        edge_type *anon_edge_type(int priority, int nolanes, double speed);

        node      *retreive_node(const str &id);
        edge_type *retreive_edge_type(const str &id);

        bool xml_read(node &n,       xmlpp::TextReader &reader);
        bool xml_read(edge_type &et, xmlpp::TextReader &reader);
        bool xml_read(edge &e,       xmlpp::TextReader &reader);

        bool xml_read_nodes(const char *filename);
        bool xml_read_nodes(xmlpp::TextReader &reader);

        bool xml_read_types(const char *filename);
        bool xml_read_types(xmlpp::TextReader &reader);

        bool xml_read_edges(const char *filename);
        bool xml_read_edges(xmlpp::TextReader &reader);

        bool check_edge(const edge &e) const;
        bool check_node(const node &n) const;
        bool check() const;
    };
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
