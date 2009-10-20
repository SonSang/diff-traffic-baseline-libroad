#include "osm_network.hpp"

namespace osm
{
    bool network::check_edge(const edge &e) const
    {
        return (e.to != e.from);
    }

    bool network::check_node(const node &n) const
    {
        return !n.id.empty();
    }

    bool network::check() const
    {
        typedef std::pair<str, node> nmap_pair;
        BOOST_FOREACH(const nmap_pair &np, nodes)
        {
            if(!check_node(np.second))
                return false;
        }

        typedef std::pair<str, edge> emap_pair;
        BOOST_FOREACH(const emap_pair &ep, edges)
        {
            if(!check_edge(ep.second))
                return false;
        }
        return true;
    }
}
