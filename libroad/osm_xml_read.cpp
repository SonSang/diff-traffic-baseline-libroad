#include "osm_network.hpp"
#include "xml_util.hpp"

//TODO
#include <iostream>

namespace osm
{


    static inline bool xml_read(network &n, node &no, xmlpp::TextReader &reader)
    {
        bool res = get_attribute(no.id,    reader, "id") &&
                   get_attribute(no.xy[0], reader, "lat")  &&
                   get_attribute(no.xy[1], reader, "lon");
        if(!res)
            return false;

        if(!get_attribute(no.type, reader, "type"))
            no.type = node::unknown;

        return true;
    }



  static inline bool xml_read(network &n, edge &e, str id, xmlpp::TextReader &reader)
    {
        bool ret;
        bool first_node = true;
        bool is_road = false;
        do
        {
          ret = read_skip_comment(reader);
          if (!ret)
            return 1;
          if (reader.get_node_type() == xmlpp::TextReader::Element){
            if (reader.get_name() == "nd")
              {

                str n_id;
                node* current_node;
                get_attribute(n_id, reader, "ref");
                current_node = retrieve<node>(n.nodes, n_id);

                e.shape.push_back(current_node);

                //Set the "to node" -- which is expected to be the last node --
                //every time.  On the final iteration, the last node will be kept.
                e.to = current_node->id;

                //Save the first node as the "from" node
                if (first_node){
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
        }while(ret && !is_closing_element(reader, "way"));


        if (!is_road){
          n.edges.erase(id);
        }


        return true;
    }

    static inline bool xml_read_nodes(network &n, xmlpp::TextReader &reader)
    {

        bool ret = read_skip_comment(reader);

        ret = ret and read_map_no_container(n, n.nodes, reader, "node", "nodes");

        return ret;
    }

    static inline bool xml_read_edges(network &n, xmlpp::TextReader &reader)
    {
        return (read_skip_comment(reader) &&
                read_map_no_container_and_children(n, n.edges, reader, "way", "nd"));
    }

    static inline bool xml_read_nodes(network &n, const char *filename)
    {
        std::cout << "xml_read_nodes1\n";
        try
        {
            xmlpp::TextReader reader(filename);

            bool res = xml_read_nodes(n, reader);

            reader.close();

            return res;
        }
        catch(const std::exception &e)
        {
            return false;
        }
    }

    static inline bool xml_read_edges(network &n, const char *filename)
    {
        std::cout << "xml_read_edges\n";

        xmlpp::TextReader reader(filename);

        bool res = (read_skip_comment(reader) &&
                    xml_read_edges(n, reader));

        reader.close();
        return res;
    }

    network load_xml_network(const char *osm_file)
    {
        network n;
        std::cout << "load_xml_network\n";

        if(!((xml_read_nodes(n, osm_file)) && (xml_read_edges(n, osm_file)))){
            std::cout << "exception thrown here\n";
            throw std::exception();
        }

        n.compute_edge_types();

        return n;
    }
}
