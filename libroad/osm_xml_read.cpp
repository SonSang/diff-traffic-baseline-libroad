#include "osm_network.hpp"
#include "xml_util.hpp"

//TODO
#include <iostream>

namespace osm
{
    static inline void xml_read(network &n, node &no, xmlpp::TextReader &reader)
    {
        get_attribute(no.id,    reader, "id");
        get_attribute(no.xy[0], reader, "lon");
        get_attribute(no.xy[1], reader, "lat");
        no.xy[2] = 0.0f;

        try
        {
            get_attribute(no.type, reader, "type");
        }
        catch(missing_attribute &ma)
        {
            no.type = node::unknown;
        }
    }

    static inline void xml_read(network &n, edge &e, xmlpp::TextReader &reader)
    {
        bool first_node = true;
        bool is_road = false;
        do
        {
            read_skip_comment(reader);

            if (reader.get_node_type() == xmlpp::TextReader::Element)
            {
                if (reader.get_name() == "nd")
                {
                    str n_id;
                    node* current_node;
                    get_attribute(n_id, reader, "ref");
                    current_node = retrieve<node>(n.nodes, n_id);

                    e.shape.push_back(current_node);
                    current_node->edges_including.push_back(&e);

                    //Set the "to node" -- which is expected to be the last node --
                    //every time.  On the final iteration, the last node will be kept.
                    e.to = current_node->id;

                    //Save the first node as the "from" node
                    if (first_node)
                    {
                        first_node = false;
                        e.from = current_node->id;
                    }
                }

                if (reader.get_name() == "tag")
                {
                    str k, v;
                    get_attribute(k, reader, "k");

                    if (k== "highway"){
                        is_road = true;
                        get_attribute(v, reader, "v");
                        e.highway_class = v;
                    }
                }
            }
        }
        while(!is_closing_element(reader, "way"));

        if (!is_road)
            n.edge_hash.erase(e.id);
    }

    static inline void xml_read_nodes(network &n, xmlpp::TextReader &reader)
    {
        read_skip_comment(reader);
        read_map_no_container(n, n.nodes, reader, "node");
    }

    static inline void xml_read_edges(network &n, xmlpp::TextReader &reader)
    {
        read_skip_comment(reader);
        read_map_no_container(n, n.edge_hash, reader, "way");
    }

    static inline void xml_read_center(network &n, const char* filename)
    {
        xmlpp::TextReader reader(filename);
        do
        {
            try
            {
                read_skip_comment(reader);
            }
            catch(xml_eof &e)
            {
                break;
            }
            if (reader.get_node_type() == xmlpp::TextReader::Element)
            {
                if (reader.get_name() == "bounds")
                {
                    float minlat, minlon, maxlat, maxlon;
                    get_attribute(minlat, reader, "minlat");
                    get_attribute(minlon, reader, "minlon");
                    get_attribute(maxlat, reader, "maxlat");
                    get_attribute(maxlon, reader, "maxlon");

                    n.center[0] = (maxlon - minlon)/2.0 + minlon;
                    n.center[1] = (maxlat - minlat)/2.0 + minlat;

                    n.topleft[0] = minlon;
                    n.topleft[1] = maxlat;
                    n.bottomright[0] = maxlon;
                    n.bottomright[1] = minlat;
                    std::cout << "Got " << minlat << " " << minlon << " " << maxlat
                              << " " << maxlon << std::endl;
                    std::cout << n.center[0] << " " << n.center[1] << std::endl;
                }
            }
        }while(1);
    }

    static inline void xml_read_nodes(network &n, const char *filename)
    {
        std::cout << "xml_read_nodes1\n";
        xmlpp::TextReader reader(filename);

        xml_read_nodes(n, reader);

        reader.close();
    }

    static inline void xml_read_edges(network &n, const char *filename)
    {
        std::cout << "xml_read_edges\n";

        xmlpp::TextReader reader(filename);
        read_skip_comment(reader);
        xml_read_edges(n, reader);

        reader.close();
    }

    network load_xml_network(const char *osm_file)
    {
        network n;
        std::cout << "load_xml_network\n";

        xml_read_center(n, osm_file);

        xml_read_nodes(n, osm_file);
        xml_read_edges(n, osm_file);

        return n;
    }
}
