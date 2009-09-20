#include <libroad/sumo_network.hpp>
#include <iostream>

#include <boost/foreach.hpp>

int main(int argc, char *argv[])
{
    sumo::network net = sumo::load_xml_network(argv[1],
                                               argv[2],
                                               argv[3]);

    {
        typedef std::pair<std::string, sumo::node> maptype;
        BOOST_FOREACH(const maptype &n, net.nodes)
        {
            std::cout << n.second.id << " " << n.second.xy << " " << n.second.type << std::endl;
        }
    }

    {
        typedef std::pair<std::string, sumo::edge_type> maptype;
        BOOST_FOREACH(const maptype &n, net.types)
        {
            std::cout << n.second.id << " " << n.second.priority << " " << n.second.nolanes << " " << n.second.speed << std::endl;
        }
    }

    {
        typedef std::pair<std::string, sumo::edge> maptype;
        BOOST_FOREACH(const maptype &n, net.edges)
        {
            std::cout << n.second.id << " " << n.second.from->id << " " << n.second.to->id << " " << n.second.type->id << " " << n.second.spread;
            BOOST_FOREACH(const vec2d &v, n.second.shape)
            {
                std::cout << " [" << v[0] << ", " << v[1] << "]";
            }
            std::cout << std::endl;
        }
    }

    if(net.check())
        std::cout << "Network checks out" << std::endl;
    else
        std::cout << "Network doesn't check out" << std::endl;

    return 0;
}
