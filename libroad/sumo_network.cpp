#include <boost/format.hpp>
#include <boost/algorithm/string.hpp>

#include "sumo_network.hpp"
#include "xml_attributes.hpp"
#include "xml_util.hpp"

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

    bool network::xml_read(node &n, xmlTextReaderPtr reader)
    {
        return read_attributes(att_list("id",   n.id,    xa::NEEDED,
                                        "x",    n.xy[0], xa::NEEDED,
                                        "y",    n.xy[1], xa::NEEDED,
                                        "type", n.type,  xa::OPTIONAL),
                               reader);
    }

    bool network::xml_read(edge_type &et, xmlTextReaderPtr reader)
    {
        return read_attributes(att_list("id",       et.id,       xa::NEEDED,
                                        "priority", et.priority, xa::NEEDED,
                                        "nolanes",  et.nolanes,  xa::NEEDED,
                                        "speed",    et.speed,    xa::NEEDED),
                               reader);
    }

    bool network::xml_read(edge &e, xmlTextReaderPtr reader)
    {
        if(!read_attributes(att_list("id", e.id, xa::NEEDED),
                            reader))
            return false;

        bool have_nodes = read_attributes(att_list("fromnode", e.from, xa::NEEDED,
                                                   "tonode",   e.to,   xa::NEEDED),
                                          reader);
        if(!have_nodes)
        {
            vec2d from;
            vec2d to;
            if(!read_attributes(att_list("xfrom", from[0], xa::NEEDED,
                                         "xfrom", from[1], xa::NEEDED,
                                         "xto",   to[0],   xa::NEEDED,
                                         "yto",   to[1],   xa::NEEDED),
                                reader))
                return false;

            e.from = anon_node(from);
            e.to   = anon_node(to);
        }
        return false;
    }

    bool network::xml_read_nodes(const char *filename)
    {
        xmlTextReaderPtr reader = xmlReaderForFile(filename,
                                                   NULL,
                                                   0);

        return reader &&
            xmlTextReaderRead(reader) == 1 &&
            xml_read_nodes(reader) &&
            xmlTextReaderClose(reader) == 0;
    }

    bool network::xml_read_nodes(xmlTextReaderPtr reader)
    {
        return is_opening_element(reader, BAD_CAST "nodes") &&
            xmlTextReaderRead(reader) == 1 &&
            read_map(*this, nodes, reader, BAD_CAST "node", BAD_CAST "nodes");
    }

    bool network::xml_read_types(const char *filename)
    {
        xmlTextReaderPtr reader = xmlReaderForFile(filename,
                                                   NULL,
                                                   0);

        return reader &&
            xmlTextReaderRead(reader) == 1 &&
            xml_read_types(reader) &&
            xmlTextReaderClose(reader) == 0;
    }

    bool network::xml_read_types(xmlTextReaderPtr reader)
    {
        return is_opening_element(reader, BAD_CAST "types") &&
            xmlTextReaderRead(reader) == 1 &&
            read_map(*this, types, reader, BAD_CAST "type", BAD_CAST "types");
    }
}

