#include <libroad/hwm_network.hpp>
#include <iostream>

#include <boost/foreach.hpp>

int main(int argc, char *argv[])
{
    std::cerr << libroad_package_string() << std::endl;
    if(argc < 5)
    {
        std::cerr << "Usage: " << argv[0] << " <node file> <edge type file> <edge file> <hwm output file>" << std::endl;
        return 1;
    }

    sumo::network snet(sumo::load_xml_network(argv[1],
                                              argv[2],
                                              argv[3]));
    std::cerr << "SUMO input net loaded successfully" << std::endl;

    hwm::network hnet(hwm::from_sumo("test", 0.5f, 2.5f, snet));

    try
    {
        hnet.check();
        std::cerr << "Derived HWM net checks out" << std::endl;
    }
    catch(std::runtime_error &e)
    {
        std::cerr << "Derived HWM net doesn't check out" << std::endl;
        exit(1);
    }

    BOOST_FOREACH(const hwm::road_pair &r, hnet.roads)
    {
        std::cout << r.second.id << " " << r.second.name << " ";
        BOOST_FOREACH(const vec3f &f, r.second.rep.points_)
        {
            std::cout << f;
        }
        std::cout << std::endl;
    }

    BOOST_FOREACH(const hwm::lane_pair &l, hnet.lanes)
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

    BOOST_FOREACH(const hwm::intersection_pair &i, hnet.intersections)
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

    hnet.xml_write(argv[4]);

    return 0;
}
