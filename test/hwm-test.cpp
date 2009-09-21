#include <libroad/hwm_network.hpp>
#include <iostream>

#include <boost/foreach.hpp>

int main(int argc, char *argv[])
{
    hwm::network net(hwm::load_xml_network(argv[1]));

    std::cerr << "HWM net loaded successfully" << std::endl;

    if(net.check())
        std::cerr << "HWM net checks out" << std::endl;
    else
        std::cerr << "HWM net doesn't check out" << std::endl;

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

    typedef std::pair<const str, hwm::intersection> imap_itr;
    BOOST_FOREACH(const imap_itr &i, net.intersections)
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
            int c = 0;
            BOOST_FOREACH(const hwm::intersection::state::out_id &oi, s.in_states)
            {
                std::cout << " " << c++ << " : " << oi.out_ref << " ";
            }
            std::cout << " out";

            c = 0;
            BOOST_FOREACH(const hwm::intersection::state::in_id &ii, s.out_states)
            {
                std::cout << " " << c++ << " : " << ii.in_ref << " ";
            }
        }
        std::cout << " } " << std::endl;
    }
    return 0;
}
