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

    typedef std::pair<const str, hwm::lane> lmap_itr;
    BOOST_FOREACH(const lmap_itr &l, net.lanes)
    {
        std::cout << l.second.id << " " << l.second.speedlimit << " {";
        typedef hwm::lane::road_membership::intervals::entry rme;
        BOOST_FOREACH(const rme &rm, l.second.road_memberships)
        {
            std::cout << rm.first << ": " << rm.second.parent_road->id << " [" << rm.second.interval[0] << "," << rm.second.interval[1] << "] " << rm.second.lane_position << " |";
        }
        std::cout << " } left {";

        typedef hwm::lane::adjacency::intervals::entry ai;
        BOOST_FOREACH(const ai &left, l.second.left)
        {
            if(left.second.neighbor)
                std::cout << left.first << ": " << left.second.neighbor->id << " [" << left.second.neighbor_interval[0] << "," << left.second.neighbor_interval[1] << "] ";
        }
        std::cout << " } right {";

        BOOST_FOREACH(const ai &right, l.second.right)
        {
            if(right.second.neighbor)
                std::cout << right.first << ": " << right.second.neighbor->id << " [" << right.second.neighbor_interval[0] << "," << right.second.neighbor_interval[1] << "] ";
        }
        std::cout << " } " << std::endl;


    }


    return 0;
}
