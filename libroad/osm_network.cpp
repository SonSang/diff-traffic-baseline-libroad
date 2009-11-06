#include "osm_network.hpp"
#include <FL/gl.h>
#include <FL/glu.h>
#include <FL/glut.H>
#include <boost/fusion/sequence.hpp>
#include <boost/fusion/include/sequence.hpp>
#include <boost/fusion/container/vector.hpp>
#include <boost/fusion/include/vector.hpp>
#include <boost/fusion/container/vector/vector_fwd.hpp>
#include <boost/fusion/include/vector_fwd.hpp>
#include <boost/fusion/container/list.hpp>
#include <boost/fusion/include/list.hpp>
#include <boost/fusion/container/list/list_fwd.hpp>
#include <boost/fusion/include/list_fwd.hpp>
#include <vector>


vec2d bias, prev;
bool first = 1;
double scale = 20.0;

using boost::fusion::at_c;

std::vector<boost::fusion::vector<double, double, double> > colors;

namespace osm
{
    typedef std::pair<const str, edge> edge_pair;
    typedef std::pair<const str, node> node_pair;

    bool network::draw_network()
    {

        glColor3f(0,0,0);

        int i = 0;

        if (first)
        {
            BOOST_FOREACH(edge_pair &ep, edges)
            {
                colors.push_back(boost::fusion::vector<double, double, double>(rand()/(double)RAND_MAX,rand()/(double)RAND_MAX,rand()/(double)RAND_MAX));
            }
        }

        BOOST_FOREACH(edge_pair &ep, edges)
        {
            edge& e = ep.second;
            shape_t& e_nodes = e.shape;

            glBegin(GL_LINES);
            BOOST_FOREACH(node& n, e_nodes)
            {
                vec2d pt = n.xy;
                if (first)
                {
                    first = 0;
                    bias = pt;
                }
                glColor3f(at_c<0>(colors[i]), at_c<1>(colors[i]), at_c<2>(colors[i]));
                glVertex3f((pt[0] - bias[0])*scale, (pt[1] - bias[1])*scale, 0);

                //Don't write the second point for the first segment of the road. (Previous point does not exist.)
                if ((pt[0] != e_nodes[0].xy[0]) and
                          (pt[1] != e_nodes[0].xy[1]))
                {
                    glVertex3f((prev[0] - bias[0])*scale, (prev[1] - bias[1])*scale, 0);
                }

                prev = pt;
            }
            glEnd();
            i++;
        }

    }

    bool network::compute_node_degrees()
    {
        BOOST_FOREACH(const osm::node_pair &np, nodes)
        {
            node_degrees.insert(std::pair<str,int>(np.first, 0));
        }

        BOOST_FOREACH(const osm::edge_pair &ep, edges)
        {
            const osm::edge &e = ep.second;

            BOOST_FOREACH(const osm::node &n, e.shape)
            {
                node_degrees[n.id]++;
            }
        }
    }

    bool network::join(osm::edge* a, osm::edge* b)
    {
        a->to = b->to;

        uint a_size = a->shape.size();
        uint b_size = b->shape.size();

        //Add all of b's nodes except the first one.
        BOOST_FOREACH(const osm::node &n, b->shape)
        {

            if (n.id != a->shape[a->shape.size() - 1].id)
                a->shape.push_back(n);
        }

        assert(a->shape.size() == a_size + b_size - 1);
    }


    bool network::split_into_road_segments()
    {
        //At every intersection, the road should be split into two segments.
        return true;
    }


    bool network::join_logical_roads()
    {
        std::vector<str> edges_to_delete;
        std::map<str, edge*> replaced_nodes;

        BOOST_FOREACH(osm::edge_pair &ep, edges)
        {
            edge& e = ep.second;
            replaced_nodes.insert(std::make_pair(e.id, &e));
        }

        BOOST_FOREACH(osm::edge_pair &ep, edges)
        {
            edge* e = &ep.second;

            //If this edge has been merged into another, load that edge.
            edge e_foo;
            do{
                e_foo = *e;
                e = replaced_nodes[e->id];
            }while(e->id != e_foo.id);

            BOOST_FOREACH(osm::edge_pair &other, edges)
            {
                edge* o = &other.second;

                //If this edge has been merged into another, load that edge.
                edge o_foo;
                do{
                    o_foo = *o;
                    o = replaced_nodes[o->id];
                }while(o->id != o_foo.id);

                //Skip if the edge is the same as the loaded edge.
                if (e->id == o->id)
                    continue;

                //We will combine two polylines if they share an endpoint and there is no third polyline connected to that point.
                if ((nodes[e->to].id == nodes[o->from].id) and (node_degrees[e->to] == 2)){
                    node_degrees[e->to]--;

                    int e_size = e->shape.size();
                    int o_size = o->shape.size();
                    join(e, o);
                    assert(e->shape.size() == e_size + o_size - 1);

                    replaced_nodes.insert(std::make_pair(o->id, e));
                    edges_to_delete.push_back(o->id);
                }

            }
        }

        //Remove edges that were merged.
        BOOST_FOREACH(const str& id, edges_to_delete)
        {
            edges.erase(id);
        }

        return true;
    }

    /*
    bool network::compute_intersections()
    {
        //Locate all split points.
        std::map<str, std::vector<str> > road_split_points;
        BOOST_FOREACH(edge_pair &ep, edges)
        {
            BOOST_FOREACH(node &node, ep.second.shape)
            {

                //Skip the first and last nodes.
                if (ep.from = node.id)
                    continue;

                if (ep.to = node.id)
                    continue;

                if (node_degrees[node.id] > 1)
                {
                    road_split_points[ep.first].push_back(node.id);
                }
            }
        }

        //Split each edge at its split points.
        std::vector<edge> new_edges;
        BOOST_FOREACH(edge_pair &ep, edges)
        {
            edge& _edge = ep.second;
            int node_index = 0;
            BOOST_FOREACH(str id, road_split_points[ep.first])
            {
                new_edges.push_back(edge());
                new_edges.end()->edge_type = _edge.edge_type;
                while (edge.nodes[node_index].id != id)
                {

                    node_index++;
                }
            }
        }




        return true;
    }
    */

    bool network::compute_edge_types()
    {
        BOOST_FOREACH( edge_pair &ep, edges)
        {
            edge& e = ep.second;
            edge_type* e_type = retrieve<edge_type>(types, ep.first);
            e.type = e_type;
            e_type->speed = 25;
            e_type->nolanes = 1;
            e_type->oneway = 0;
            if(e.highway_class == "residential")
            {
                e_type->speed = 30;
                e_type->nolanes = 1;
                e_type->oneway = 0;
            }
            if(e.highway_class == "primary")
            {
                e_type->nolanes = 2;
                e_type->oneway = 0;
                e_type->speed = 50;
            }
            if(e.highway_class == "secondary")
            {
                e_type->speed = 40;
                e_type->nolanes = 2;
                e_type->oneway = 0;
            }
            if(e.highway_class == "service")
            {
                e_type->speed = 25;
                e_type->nolanes = 1;
                e_type->oneway = 0;
            }
            if(e.highway_class == "primary_link")
            {
                e_type->speed = 30;
                e_type->nolanes = 1;
                e_type->oneway = 1;
            }
            if(e.highway_class == "secondary_link")
            {
                e_type->speed = 30;
            e_type->nolanes = 1;
            e_type->oneway = 1;
            }

        }
    }
}
