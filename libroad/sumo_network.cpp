#include <boost/format.hpp>
#include <boost/algorithm/string.hpp>
#include <iostream>
#include "sumo_network.hpp"

namespace sumo
{
    node::id_t network::anon_node(vec2d &pos)
    {
        node::id_t id(boost::str(boost::format("anon_node%u") % anon_node_count++));

        node n;
        n.id = id;
        n.xy = pos;
        n.type = node::unknown;
        nodes[id] = n;

        return id;
    }

    bool network::xml_read(node &n, xmlpp::TextReader &reader)
    {
        bool res = get_attribute(n.id,    reader, "id") &&
                   get_attribute(n.xy[0], reader, "x")  &&
                   get_attribute(n.xy[1], reader, "y");
        if(!res)
            return false;

        if(!get_attribute(n.type, reader, "type"))
            n.type = node::unknown;

        return true;
    }

    bool network::xml_read(edge_type &et, xmlpp::TextReader &reader)
    {
        return (get_attribute(et.id,       reader, "id")       &&
                get_attribute(et.priority, reader, "priority") &&
                get_attribute(et.nolanes,  reader, "nolanes")  &&
                get_attribute(et.speed,    reader, "speed"));
    }

    bool network::xml_read(edge &e, xmlpp::TextReader &reader)
    {
        if(!get_attribute(e.id, reader, "id"))
            return false;

        bool have_nodes = get_attribute(e.from, reader, "fromnode") &&
                          get_attribute(e.to,   reader, "tonode");

        if(!have_nodes)
        {
            vec2d from;
            vec2d to;
            if(!(get_attribute(from[0], reader, "xfrom") &&
                 get_attribute(from[1], reader, "yfrom") &&
                 get_attribute(to[0],   reader, "xto")   &&
                 get_attribute(to[1],   reader, "yto")))
                return false;

            e.from = anon_node(from);
            e.to   = anon_node(to);
        }
        return false;
    }

    bool network::xml_read_nodes(const char *filename)
    {
        xmlpp::TextReader reader(filename);

        bool res = (reader.read() &&
                    xml_read_nodes(reader));

        reader.close();

        return res;
    }

    bool network::xml_read_nodes(xmlpp::TextReader &reader)
    {
        return (is_opening_element(reader, "nodes") &&
                reader.read() &&
                read_map(*this, nodes, reader, "node", "nodes"));
    }

    bool network::xml_read_types(const char *filename)
    {
        xmlpp::TextReader reader(filename);

        bool res = (reader.is_valid() &&
                    reader.read() &&
                    xml_read_types(reader));

        reader.close();

        return res;
    }

    bool network::xml_read_types(xmlpp::TextReader &reader)
    {
        return (is_opening_element(reader, "types") &&
                reader.read() &&
                read_map(*this, types, reader, "type", "types"));
    }
}

