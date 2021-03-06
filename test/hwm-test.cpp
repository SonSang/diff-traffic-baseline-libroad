#include <libroad/hwm_network.hpp>
#include <iostream>

#include <boost/foreach.hpp>

int main(int argc, char *argv[])
{
    std::cerr << libroad_package_string() << std::endl;
    if(argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " <network file>" << std::endl;
        return 1;
    }

    hwm::network net(hwm::load_xml_network(argv[1]));

    std::cerr << "HWM net loaded successfully" << std::endl;

    try
    {
        net.check();
        std::cerr << "HWM net checks out" << std::endl;
    }
    catch(std::runtime_error &e)
    {
        std::cerr << "HWM net doesn't check out: " << e.what() << std::endl;
        exit(1);
    }

    BOOST_FOREACH(const hwm::road_pair &r, net.roads)
    {
        std::cout << r.second.id << " " << r.second.name << " ";
        BOOST_FOREACH(const vec3f &f, r.second.rep.points_)
        {
            std::cout << f;
        }
        std::cout << std::endl;
    }

    BOOST_FOREACH(const hwm::lane_pair &l, net.lanes)
    {
        std::cout << l.second.id << " " << l.second.speedlimit << " {";
        typedef hwm::lane::road_membership::intervals::entry rme;
        BOOST_FOREACH(const rme &rm, l.second.road_memberships)
        {
            std::cout << rm.first << ": " << rm.second.parent_road->id << " " << rm.second.interval << " " << rm.second.lane_position << " |";
        }
        std::cout << " } left {";

        typedef hwm::lane::adjacency::intervals::entry ai;
        BOOST_FOREACH(const ai &left, l.second.left)
        {
            if(left.second.neighbor)
                std::cout << left.first << ": " << left.second.neighbor->id << " " << left.second.neighbor_interval << " ";
        }
        std::cout << " } right {";

        BOOST_FOREACH(const ai &right, l.second.right)
        {
            if(right.second.neighbor)
                std::cout << right.first << ": " << right.second.neighbor->id << " " << right.second.neighbor_interval << " ";
        }
        std::cout << " } " << std::endl;
    }

    BOOST_FOREACH(const hwm::intersection_pair &i, net.intersections)
    {
        std::cout << i.second.id << " incoming {";
        BOOST_FOREACH(const hwm::lane* la, i.second.incoming)
        {
            std::cout << la->id << " ";
        }
        std::cout << " } outgoing {";

        BOOST_FOREACH(const hwm::lane* la, i.second.outgoing)
        {
            std::cout << la->id << " ";
        }
        std::cout << " } states {";

        int id = 0;
        BOOST_FOREACH(const hwm::intersection::state &s, i.second.states)
        {
            std::cout << " " << id++ << " " << s.duration << " in";
            BOOST_FOREACH(const hwm::intersection::state::state_pair &sp, s.in_pair())
            {
                std::cout << "in: " << sp.in_idx << " out: " << sp.out_idx << " fict_lane: " << sp.fict_lane->id << std::endl;
            }
        }
        std::cout << " } " << std::endl;
    }

    net.translate(vec3f(0.0, 1.0, 0.0));

    net.xml_write(argv[2]);

    return 0;
}
