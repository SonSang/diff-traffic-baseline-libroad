#include "hwm_network.hpp"

namespace hwm
{
    void network::build_spatial()
    {
        road_space.build(roads);
    }

    road_spatial::entry::entry() : r(0), feature(0)
    {
    }

    road_spatial::entry::entry(road *in_r, int in_f) : r(in_r), feature(in_f)
    {
    }

    road_spatial::road_spatial() : tree(0)
    {
    }

    road_spatial::~road_spatial()
    {
        if(tree)
            delete tree;
    }

    void road_spatial::build(road_map &roads)
    {
        if(tree)
            delete tree;

        std::vector<rtree2d::entry> leaves;
        BOOST_FOREACH(road_pair &rp, roads)
        {
            const arc_road &ar = rp.second.rep;
            for(size_t f = 0; f < 2*ar.frames_.size() + 1; ++f)
            {
                const aabb2d rect(ar.bound_feature2d(f));

                leaves.push_back(rtree2d::entry(rect, items.size()));
                items.push_back(entry(&(rp.second), f));
            }
        }

        tree = rtree2d::hilbert_rtree(leaves);
    }

    std::vector<road_spatial::entry> road_spatial::query(const aabb2d &rect) const
    {
        std::vector<entry> res;
        if(tree)
        {
            const std::vector<size_t> q_res(tree->query(rect));
            BOOST_FOREACH(const size_t &i, q_res)
            {
                res.push_back(items[i]);
            }
        }

        return res;
    }
}
