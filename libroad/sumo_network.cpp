#include <boost/format.hpp>
#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/tokenizer.hpp>
#include <iostream>
#include "sumo_network.hpp"

namespace sumo
{
    void read_shape(edge::shape_t &shape, const std::string &s)
    {
        typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
        boost::char_separator<char> sep(" ");
        tokenizer                   tokens(s, sep);

        BOOST_FOREACH(const std::string &vs, tokens)
        {
            boost::char_separator<char> csep(",");
            tokenizer                   vtokens(vs, csep);
            tokenizer::iterator         tok        = vtokens.begin();

            double x = boost::lexical_cast<double>(*tok);
            ++tok;
            double y = boost::lexical_cast<double>(*tok);
            shape.push_back(vec2d(x, y));
        }
    }

    node::id_t network::anon_node(vec2d &pos)
    {
        node::id_t id(boost::str(boost::format("anon_node%u") % anon_node_count++));

        node n;
        n.id      = id;
        n.xy      = pos;
        n.type    = node::unknown;
        nodes[id] = n;

        return id;
    }

    edge_type::id_t network::anon_edge_type(const int priority, const int nolanes, const double speed)
    {
        edge_type::id_t id(boost::str(boost::format("anon_edge_type%u") % anon_edge_type_count++));

        edge_type et;
        et.id       = id;
        et.priority = priority;
        et.nolanes  = nolanes;
        et.speed    = speed;
        types[id]   = et;

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

        if(!(get_attribute(e.from, reader, "fromnode") &&
             get_attribute(e.to,   reader, "tonode")))
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

        if(!get_attribute(e.type, reader, "type"))
        {
            int priority;
            int nolanes;
            double speed;
            if(!(get_attribute(priority, reader, "priority") &&
                 get_attribute(nolanes, reader, "nolanes") &&
                 get_attribute(speed, reader, "speed")))
                return false;

            e.type = anon_edge_type(priority, nolanes, speed);
        }

        if(!get_attribute(e.spread, reader, "spread"))
            e.spread = edge::right;

        str shape_str;
        if(get_attribute(shape_str, reader, "shape"))
            read_shape(e.shape, shape_str);

        return true;
    }

    bool network::xml_read_nodes(const char *filename)
    {
        try
        {
            xmlpp::TextReader reader(filename);

            bool res = (read_skip_comment(reader) &&
                        xml_read_nodes(reader));

            reader.close();

            return res;
        }
        catch(const std::exception &e)
        {
            return false;
        }
    }

    bool network::xml_read_nodes(xmlpp::TextReader &reader)
    {
        return (is_opening_element(reader, "nodes") &&
                read_skip_comment(reader) &&
                read_map(*this, nodes, reader, "node", "nodes"));
    }

    bool network::xml_read_types(const char *filename)
    {
        try
        {
            xmlpp::TextReader reader(filename);

            bool res = (read_skip_comment(reader) &&
                        xml_read_types(reader));

            reader.close();
            return res;
        }
        catch(const std::exception &e)
        {
            return false;
        }
    }

    bool network::xml_read_types(xmlpp::TextReader &reader)
    {
        return (is_opening_element(reader, "types") &&
                read_skip_comment(reader) &&
                read_map(*this, types, reader, "type", "types"));
    }

    bool network::xml_read_edges(const char *filename)
    {
        try
        {
            xmlpp::TextReader reader(filename);

            bool res = (read_skip_comment(reader) &&
                        xml_read_edges(reader));

            reader.close();
            return res;
        }
        catch(const std::exception &e)
        {
            return false;
        }
    }

    bool network::xml_read_edges(xmlpp::TextReader &reader)
    {
        return (is_opening_element(reader, "edges") &&
                read_skip_comment(reader) &&
                read_map(*this, edges, reader, "edge", "edges"));
    }

    bool network::check_edge(const edge &e) const
    {
        return (e.to != e.from &&
                nodes.find(e.to) != nodes.end() &&
                nodes.find(e.from) != nodes.end() &&
                types.find(e.type) != types.end());
    }

    bool network::check() const
    {
        typedef std::pair<edge::id_t,edge> emap_pair;
        BOOST_FOREACH(const emap_pair &ep, edges)
        {
            if(!check_edge(ep.second))
                return false;
        }
        return true;
    }
}
