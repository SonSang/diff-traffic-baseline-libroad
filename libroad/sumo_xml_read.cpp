#include "sumo_network.hpp"
#include "xml_util.hpp"

namespace sumo
{
    static inline void read_shape(edge::shape_t &shape, const std::string &s)
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

    node *anon_node(network &n, vec2d &pos)
    {
        str id(boost::str(boost::format("anon_node%u") % n.anon_node_count++));

        node no;
        no.id      = id;
        no.xy      = pos;
        no.type    = node::unknown;
        n.nodes[id] = no;

        return &(n.nodes[id]);
    }

    edge_type *anon_edge_type(network &n, const int priority, const int nolanes, const double speed)
    {
        str id(boost::str(boost::format("anon_edge_type%u") % n.anon_edge_type_count++));

        edge_type et;
        et.id       = id;
        et.priority = priority;
        et.nolanes  = nolanes;
        et.speed    = speed;
        n.types[id]   = et;

        return &(n.types[id]);
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

    static inline bool xml_read(network &n, node &no, xmlpp::TextReader &reader)
    {
        get_attribute(no.id,    reader, "id");
        get_attribute(no.xy[0], reader, "x");
        get_attribute(no.xy[1], reader, "y");

        try
        {
            get_attribute(no.type, reader, "type");
        }
        catch(missing_attribute &ma)
        {
            no.type = node::unknown;
        }

        return true;
    }

    static inline bool xml_read(network &n, edge_type &et, xmlpp::TextReader &reader)
    {
        get_attribute(et.id,       reader, "id");
        get_attribute(et.priority, reader, "priority");
        get_attribute(et.nolanes,  reader, "nolanes");
        get_attribute(et.speed,    reader, "speed");

        return true;
    }

    static inline bool xml_read(network &n, edge &e, xmlpp::TextReader &reader)
    {
        get_attribute(e.id, reader, "id");

        str from_id;
        str to_id;

        try
        {
            get_attribute(from_id, reader, "fromnode");
            get_attribute(to_id,   reader, "tonode");
            e.from = retrieve<node>(n.nodes, from_id);
            e.to   = retrieve<node>(n.nodes, to_id);
        }
        catch(missing_attribute &mi)
        {
            vec2d from;
            vec2d to;
            get_attribute(from[0], reader, "xfrom");
            get_attribute(from[1], reader, "yfrom");
            get_attribute(to[0],   reader, "xto");
            get_attribute(to[1],   reader, "yto");

            e.from = anon_node(n, from);
            e.to   = anon_node(n, to);
        }

        try
        {
            str type_id;
            get_attribute(type_id, reader, "type");
            e.type = retrieve<edge_type>(n.types, type_id);
        }
        catch(missing_attribute &mi)
        {
            int priority;
            int nolanes;
            double speed;
            get_attribute(priority, reader, "priority");
            get_attribute(nolanes, reader, "nolanes");
            get_attribute(speed, reader, "speed");

            e.type = anon_edge_type(n, priority, nolanes, speed);
        }

        try
        {
            get_attribute(e.spread, reader, "spread");
        }
        catch(missing_attribute &mi)
        {
            e.spread = edge::right;
        }

        str shape_str;
        try
        {
            get_attribute(shape_str, reader, "shape");
            read_shape(e.shape, shape_str);
        }
        catch(missing_attribute &mi)
        {
        }

        return true;
    }

    static inline bool xml_read_nodes(network &n, xmlpp::TextReader &reader)
    {
        assert(is_opening_element(reader, "nodes"));
        read_skip_comment(reader);
        sumo_read_map(n, n.nodes, reader, "node", "nodes");
        return true;
    }

    static inline bool xml_read_types(network &n, xmlpp::TextReader &reader)
    {
        assert(is_opening_element(reader, "types"));
        read_skip_comment(reader);
        sumo_read_map(n, n.types, reader, "type", "types");
        return true;
    }


    static inline bool xml_read_edges(network &n, xmlpp::TextReader &reader)
    {
        assert(is_opening_element(reader, "edges"));
        read_skip_comment(reader);
        sumo_read_map(n, n.edges, reader, "edge", "edges");
        return true;
    }

    static inline bool xml_read_nodes(network &n, const char *filename)
    {
        xmlpp::TextReader reader(filename);
        read_skip_comment(reader);
        bool res = xml_read_nodes(n, reader);
        reader.close();
        return res;
    }

    static inline bool xml_read_types(network &n, const char *filename)
    {
        xmlpp::TextReader reader(filename);
        read_skip_comment(reader);
        bool res = xml_read_types(n, reader);

        reader.close();
        return res;
    }

    static inline bool xml_read_edges(network &n, const char *filename)
    {
        xmlpp::TextReader reader(filename);
        read_skip_comment(reader);
        bool res = xml_read_edges(n, reader);
        reader.close();
        return res;
    }

    network load_xml_network(const char *node_file,
                             const char *edge_type_file,
                             const char *edge_file)
    {
        network n;

        if(!(xml_read_nodes(n, node_file) &&
             xml_read_types(n, edge_type_file) &&
             xml_read_edges(n, edge_file)))
            throw std::exception();

        return n;
    }
}
