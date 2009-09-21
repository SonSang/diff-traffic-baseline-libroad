#include <boost/multi_index_container.hpp>
#include <boost/multi_index/member.hpp>
#include <boost/multi_index/ordered_index.hpp>
#include <iostream>

struct lane_pair
{
    lane_pair(int s, int d, void *fl) : src(s), dest(d), fict_lane(fl)
    {}

    int src;
    int dest;
    void *fict_lane;
};

struct src {};
struct dest {};

using boost::multi_index_container;
using namespace boost::multi_index;

typedef multi_index_container<
    lane_pair,
    indexed_by<
        ordered_unique<
            tag<src>,  BOOST_MULTI_INDEX_MEMBER(lane_pair,int,src)>,
        ordered_unique<
            tag<dest>, BOOST_MULTI_INDEX_MEMBER(lane_pair,int,dest)>
        >
    > lane_pairs;

int main()
{
    lane_pairs lp;

    lp.insert(lane_pair(0, 1, (void*)0x4));
    lp.insert(lane_pair(1, 0, (void*)0x4));

    lane_pairs::iterator i0 = get<src>(lp).find(0);
    lane_pairs::iterator i1 = get<src>(lp).find(1);

    std::cout << "I: " << i0->src << std::endl;
    std::cout << "I: " << i1->src << std::endl;

    return 0;
};
