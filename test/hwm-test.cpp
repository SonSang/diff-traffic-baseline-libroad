#include <libroad/hwm_network.hpp>
#include <iostream>

#include <boost/foreach.hpp>

int main(int argc, char *argv[])
{
    hwm::network net;

    if(!net.xml_read(argv[1]))
    {
        std::cerr << "HWM net failed to load: " << argv[1] << std::endl;
        return 1;
    }
    else
        std::cerr << "HWM net loaded successfully" << std::endl;

    typedef std::pair<const str, hwm::road> rmap_itr;
    BOOST_FOREACH(const rmap_itr &r, net.roads)
    {
        std::cout << r.second.id << " " << r.second.name << " ";
        BOOST_FOREACH(const vec3f &f, r.second.rep.points_)
        {
            std::cout << f;
        }
        std::cout << std::endl;
    }

    return 0;
}
