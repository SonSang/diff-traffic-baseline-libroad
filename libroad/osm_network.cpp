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
#include <sstream>


vec2d bias, prev;
bool first_for_display = 1;
double scale = 20.0;


using boost::fusion::at_c;

std::vector<boost::fusion::vector<double, double, double> > colors;

namespace osm
{
    //Instantiate static

    size_t network::new_edges_id = 0;


    typedef std::pair<const str, edge> edge_pair;
    typedef std::pair<const str, node> node_pair;
    typedef std::pair<const str, intersection> intr_pair;

    bool network::draw_network()
    {

        glColor3f(0,0,0);

        int i = 0;

        if (first_for_display)
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

            glBegin(GL_LINE_STRIP);
            glColor3f(at_c<0>(colors[i]),
                      at_c<1>(colors[i]),
                      at_c<2>(colors[i]));
            vec2d pt = e_nodes[0].xy;
            if (first_for_display)
            {
                first_for_display = 0;
                bias = pt;
            }

            glVertex3f((pt[0] - bias[0])*scale,
                       (pt[1] - bias[1])*scale,
                       0);
            for(int i = 1; i < e_nodes.size(); i++)
            {
                glVertex3f((e_nodes[i].xy[0] - bias[0])*scale,
                           (e_nodes[i].xy[1] - bias[1])*scale,
                           0);
            }
            // BOOST_FOREACH(node& n, e_nodes)
            // {
            //     vec2dg pt = n.xy;
            //     if (first_for_display)
            //     {
            //         first_for_display = 0;
            //         bias = pt;
            //     }
            //     glColor3f(at_c<0>(colors[i]), at_c<1>(colors[i]), at_c<2>(colors[i]));
            //     glVertex3f((pt[0] - bias[0])*scale, (pt[1] - bias[1])*scale, 0);

            //     //Don't write the second point for the first segment of the road. (Previous point does not exist.)
            //     if ((pt[0] != e_nodes[0].xy[0]) and
            //               (pt[1] != e_nodes[0].xy[1]))
            //     {
            //         glVertex3f((prev[0] - bias[0])*scale, (prev[1] - bias[1])*scale, 0);
            //     }

            //     prev = pt;
            // }
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
        assert(a->shape[0].id == a->from);
        assert(a->shape[a->shape.size() - 1].id == a->to);
        assert(b->shape[0].id == b->from);
        assert(b->shape[b->shape.size() - 1].id == b->to);
        assert(a->to == b->from);

        uint a_size = a->shape.size();
        uint b_size = b->shape.size();

        //Add all of b's nodes except the first one.
        for (int i = 1; i < b->shape.size(); i++)
        {
            a->shape.push_back(b->shape[i]);
        }

        a->to = b->to;

        assert(a->shape.size() == a_size + b_size - 1);
    }


    bool network::create_intersections()
    {
        BOOST_FOREACH(const osm::edge_pair &ep, edges)
        {
            const osm::edge e = ep.second;
            if (node_degrees[e.to] > 1)
            {
                intersection* curr;
                curr = retrieve<intersection>(intersections, e.to);
                curr->edges_ending_here.push_back(e.id);
            }
            if (node_degrees[e.from] > 1)
            {
                intersection* curr;
                curr = retrieve<intersection>(intersections, e.from);
                curr->edges_starting_here.push_back(e.id);
            }
        }


        double tmp_offset = 0.005;
        BOOST_FOREACH(const osm::intr_pair &ip, intersections)
        {
            const intersection& i = ip.second;
            std::cout << "starting count " << i.edges_starting_here.size() << std::endl;
            std::cout << "ending count " << i.edges_ending_here.size() << std::endl;

            BOOST_FOREACH(const str& edge_id, i.edges_starting_here)
            {
                vec2d start_seg = edges[edge_id].shape[1].xy;
                start_seg -= edges[edge_id].shape[0].xy;

                double len = sqrt(start_seg[0]*start_seg[0] + start_seg[1]*start_seg[1]);
                double factor = (len - tmp_offset)/len;
                start_seg *= factor;
                edges[edge_id].shape[0].xy[0] = edges[edge_id].shape[1].xy[0] - start_seg[0];
                edges[edge_id].shape[0].xy[1] = edges[edge_id].shape[1].xy[1] - start_seg[1];

            }
            BOOST_FOREACH(const str& edge_id, i.edges_ending_here)
            {
                vec2d end_seg = (edges[edge_id].shape.end()--)->xy;

                end_seg -=  ((edges[edge_id].shape.end()--)--)->xy;


                double len = sqrt(end_seg[0]*end_seg[0] + end_seg[1]*end_seg[1]);
                double factor = (len - tmp_offset)/len;
                std::cout << end_seg << std::endl;
                std::cout << factor << std::endl;
                end_seg *= factor;
                std::cout << end_seg << std::endl;

                (edges[edge_id].shape.end()--)->xy[0] = ((edges[edge_id].shape.end()--)--)->xy[0] + end_seg[0];
                (edges[edge_id].shape.end()--)->xy[1] = ((edges[edge_id].shape.end()--)--)->xy[1] + end_seg[1];


            }
        }



        return true;
    }


    bool network::join_logical_roads()
    {
        std::vector<str> edges_to_delete;
        std::map<const str, edge*> replaced_nodes;


        BOOST_FOREACH(osm::edge_pair &ep, edges)
        {
            replaced_nodes.insert(std::make_pair(ep.first, &ep.second));
        }


        BOOST_FOREACH(osm::edge_pair &ep, edges)
        {

            edge* e = &ep.second;
            //assert(e->to != e->from);
            assert(e->to == (--e->shape.end())->id);
            assert(e->from == (e->shape.begin())->id);

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

    edge network::copy_no_shape(const edge& e)
    {
        edge to_return;
        to_return.type = e.type;


        //Initialized to values that must be changed.
        to_return.from = "-1";
        to_return.to = "-1";

        std::stringstream sout;
        sout << network::new_edges_id;
        to_return.id = str(sout.str());
        network::new_edges_id++;

        to_return.highway_class = e.highway_class;

        return to_return;
    }


    bool network::split_into_road_segments()
    {

        //Locate all split points.
        std::map<str, std::vector<str> > road_split_points;
        BOOST_FOREACH(edge_pair &ep, edges)
        {
            BOOST_FOREACH(node &node, ep.second.shape)
             {

                //Skip the first and last nodes.
                if (ep.second.from == node.id)
                    continue;

                if (ep.second.to == node.id)
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
            std::cout << "Next edge " << std::endl;
            edge& _edge = ep.second;
            int node_index = 0;
            int _edge_ending_node = 0;

            //Find first splitter.
            bool _first = true;
            BOOST_FOREACH(str id, road_split_points[ep.first])
            {
                std::cout << "Next split point " << std::endl;

                //Skip the first splitter if it's the first point of the road.
                if (id == _edge.shape[0].id){
                    continue;
                }

                if (not _first)
                 {
                    new_edges.push_back(copy_no_shape(_edge));
                    (--new_edges.end())->from = _edge.shape[node_index].id;
                    (--new_edges.end())->shape.push_back(nodes[_edge.shape[node_index].id]);
                }

                //Add each node to new edge up to, and including, the next splitter
                //Set the first node of the new edge
                std::cout << id << " " << _edge.shape[node_index].id << std::endl;

                while (_edge.shape[node_index].id != id)
                {
                    node_index++;
                    std::cout << id << " " << _edge.shape[node_index].id << std::endl;
                    if (not _first)
                    {
                        //Add nodes to new road
                        (--new_edges.end())->shape.push_back(nodes[_edge.shape[node_index].id]);
                        (--new_edges.end())->to = _edge.shape[node_index].id;
                    }
                }

                if (not _first)
                    assert((--new_edges.end())->shape.size() > 1);

                //Node index is at the index of the split point.
                if (_first)
                {
                    std::cout << "setting edge_ending here " << node_index << std::endl;

                    //Update the node this road goes up to.
                    _edge.to = _edge.shape[node_index].id;

                    _edge_ending_node = node_index;

                    _first = false;
                }

            }

            if (road_split_points[ep.first].size() > 0)
            {
                //Now node_index is on the final splitter.
                //Add that splitter and all remaining nodes to a new edge.
                new_edges.push_back(copy_no_shape(_edge));
                (--new_edges.end())->from = _edge.shape[node_index].id;
                for (;node_index < _edge.shape.size(); node_index++){
                    (--new_edges.end())->shape.push_back(_edge.shape[node_index]);
                    (--new_edges.end())->to = _edge.shape[node_index].id;
                }
                assert((--new_edges.end())->shape.size() > 1);
                //Remove the deleted nodes from the original edge
                _edge.shape.erase(_edge.shape.begin()+ _edge_ending_node + 1, _edge.shape.end());
            }
            assert(_edge.shape.size() > 1);

        }

        //Add all created edges
        BOOST_FOREACH(edge& e, new_edges)
        {
            edges.insert(std::make_pair(e.id, e));
        }

        return true;
    }


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

        BOOST_FOREACH(osm::edge_pair &ep, edges)
        {
            edge& e = ep.second;
            assert(e.type->nolanes > 0);
            assert(e.type->nolanes < 10);
        }


    }
}
