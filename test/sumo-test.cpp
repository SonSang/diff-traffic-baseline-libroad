#include <libroad/sumo_network.hpp>
#include <iostream>

#include <boost/foreach.hpp>

int main(int argc, char *argv[])
{
    sumo::network net;
    bool res = net.xml_read_nodes(argv[1]);

    typedef std::pair<std::string, sumo::node> maptype;
    BOOST_FOREACH(const maptype &n, net.nodes)
    {
        std::cout << n.second.id << " " << n.second.xy << " " << n.second.type << std::endl;
    }

    if(!res)
        std::cerr << "Didn't load correctly!"<< std::endl;


    return 0;
}
