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
#include <limits>


vec2d bias, prev;
bool first_for_display = 1;
double scale = 157253.2964;

using boost::fusion::at_c;

std::vector<boost::fusion::vector<double, double, double> > colors;

namespace osm
{
    //Instantiate static

    size_t network::new_edges_id = 0;


    typedef std::pair<const str, edge> edge_pair;
    typedef std::pair<const str, node> node_pair;
    typedef std::pair<const str, intersection> intr_pair;

    bool network::populate_edges_from_hash()
    {
        BOOST_FOREACH(strhash<edge>::type::value_type& hash, edge_hash)
        {
            edges.push_back(hash.second);
        }
    }

    bool network::populate_edge_hash_from_edges()
    {
        edge_hash.erase(edge_hash.begin(), edge_hash.end());

        BOOST_FOREACH(edge& e, edges)
        {
            edge_hash.insert(std::make_pair(e.id, e));
        }
    }


    float edge::length() const
    {
        float len_thus_far = 0;
        float _len = 0;
        for(int i = 0; i < shape.size() - 1; i++)
        {
            vec2d start = shape[i + 1]->xy;
            start -= shape[i]->xy;
            _len = sqrt(start[0]*start[0] + start[1]*start[1]);
            len_thus_far += _len;
        }

        return len_thus_far;
    }

    bool network::remove_small_roads(double min_len)
    {
        for(int i = 0; i < edges.size(); )
        {
            edge& e = edges[i];
            if (e.length() < min_len)
            {
                //Update intersections
                if (intersections.find(e.to) != intersections.end())
                {
                    intersection& i_to = intersections[e.to];
                    i_to.edges_ending_here.erase(find(i_to.edges_ending_here.begin(), i_to.edges_ending_here.end(), &e));
                }

                if (intersections.find(e.from) != intersections.end())
                {
                    intersection& i_from = intersections[e.from];
                    i_from.edges_starting_here.erase(find(i_from.edges_starting_here.begin(), i_from.edges_starting_here.end(), &e));
                }

                //Update degree count
                node_degrees[e.to]--;
                node_degrees[e.from]--;

                //TODO join roads that this edge connected..
                std::swap(edges[i], edges.back());
                edges.pop_back();
            }
            else
                ++i;
        }
    }

    bool network::draw_network()
    {
        glColor3f(0,0,0);

        int i = 0;

        if (first_for_display)
        {
            BOOST_FOREACH(edge &ep, edges)
            {
                colors.push_back(boost::fusion::vector<double, double, double>(rand()/(double)RAND_MAX,rand()/(double)RAND_MAX,rand()/(double)RAND_MAX));
            }
        }

        BOOST_FOREACH(edge &e, edges)
        {
            shape_t& e_nodes = e.shape;

            glBegin(GL_LINE_STRIP);
            glColor3f(at_c<0>(colors[i]),
                      at_c<1>(colors[i]),
                      at_c<2>(colors[i]));

            for(int j = 0; j < e_nodes.size(); j++)
            {

                glVertex3f((e_nodes[j]->xy[0]),
                           (e_nodes[j]->xy[1]),
                           0);
            }
            glEnd();

            i++;
        }

        glPointSize(5);
        glColor3f(1,0,0);
        glBegin(GL_POINTS);
        BOOST_FOREACH(strhash<intersection>::type::value_type& i, intersections)
        {
            osm::node& n = nodes[i.second.id_from_node];
            glVertex3f(n.xy[0], n.xy[1], 0.0);
        }
        glEnd();

        first_for_display = false;

    }

    bool network::create_grid(int w, int h, double dw, double dh)
    {
        std::vector<std::vector< node*> > node_grid (w, std::vector<node*>(h));

        for(int i = 0; i < w; i++)
        {
            for(int j = 0; j < h; j++)
            {
                std::stringstream node_id;
                node_id << "node " << i << "_" << j;
                node* n = retrieve<node>(nodes, str(node_id.str()));
                n->id = str(node_id.str());
                n->xy[0] = i*dw;
                n->xy[1] = j*dh;

                node_grid[i][j] = n;

                if (j != 0) //create vertical edge
                {
                    str e_id = n->id+"to"+node_grid[i][j-1]->id;

                    edge* e = NULL;
                    for(int k=0; k < edges.size(); k++)
                    {
                        if (edges[k].id == e_id)
                        {
                            e = &edges[k];
                            break;
                        }
                    }
                    if (e == NULL)
                    {
                        edges.push_back(edge());
                        e = &edges.back();
                    }
                    e->id = e_id;
                    e->from = n->id;
                    e->to = node_grid[i][j-1]->id;
                    e->highway_class = "urban";
                    e->shape.push_back(&nodes[e->from]);
                    e->shape.push_back(&nodes[e->to]);
                }

                if (i != 0)
                {
                    str e_id = n->id+"to"+node_grid[i-1][j]->id;
                    edge* e = NULL;
                    for(int k=0; k < edges.size(); k++)
                    {
                        if (edges[k].id == e_id)
                        {
                            e = &edges[k];
                            break;
                        }
                    }
                    if (e == NULL)
                    {
                        edges.push_back(edge());
                        e = &edges.back();
                    }
                    e->id = e_id;
                    e->from = n->id;
                    e->to = node_grid[i-1][j]->id;
                    e->highway_class = "urban";
                    e->shape.push_back(&nodes[e->from]);
                    e->shape.push_back(&nodes[e->to]);
                }
            }
        }
    }

    bool network::compute_node_degrees()
    {
        BOOST_FOREACH(const osm::node_pair &np, nodes)
        {
            node_degrees.insert(std::pair<str,int>(np.first, 0));
        }

        BOOST_FOREACH(const osm::edge &e, edges)
        {
            BOOST_FOREACH(const osm::node *n, e.shape)
            {
                node_degrees[n->id]++;
            }
        }
    }

    {
        {

    }

    bool network::create_intersections()
    {
        BOOST_FOREACH(osm::edge &e, edges)
        {
            if (node_degrees[e.to] > 1)
            {
                intersection* curr;
                curr = retrieve<intersection>(intersections, e.to);
                curr->edges_ending_here.push_back(&e);
                curr->id_from_node = e.to;
            }
            if (node_degrees[e.from] > 1)
            {
                intersection* curr;
                curr = retrieve<intersection>(intersections, e.from);
                curr->edges_starting_here.push_back(&e);
                curr->id_from_node = e.from;
            }
        }

        //Pull back roads to make room for intersections.
        //TODO use geometric method to create exact intersection geometry.
        double tmp_offset = 10;
        BOOST_FOREACH(const osm::intr_pair &ip, intersections)
        {
            const intersection& i = ip.second;

            BOOST_FOREACH(edge* edge_p, i.edges_starting_here)
            {
                edge& e = (*edge_p);

                //TODO Put the min angle calculations in a seperate function
                tvmet::Vector<float, 2> alpha(e.shape[1]->xy[0] - e.shape[0]->xy[0],
                                              e.shape[1]->xy[1] - e.shape[0]->xy[1]);
                alpha *= 1.0/tvmet::norm2(alpha);

                double min_angle = std::numeric_limits<double>::infinity();
                BOOST_FOREACH(edge* other_edge_p, i.edges_starting_here)
                {
                    edge& f = (*other_edge_p);

                    if (e.id == f.id)
                        continue;

                    tvmet::Vector<float, 2> beta(f.shape[1]->xy[0] - f.shape[0]->xy[0],
                                                 f.shape[1]->xy[1] - f.shape[0]->xy[1]);
                    beta *= 1.0/tvmet::norm2(beta);

                    float angle = acos(dot(alpha, beta));

                    if (angle < min_angle)
                    {
                        min_angle = angle;
                    }
                }

                BOOST_FOREACH(edge* other_edge_p, i.edges_ending_here)
                {
                    edge& f = (*other_edge_p);

                    tvmet::Vector<float, 2> beta;
                    beta = f.shape[f.shape.size() - 2]->xy;
                    beta -= f.shape[f.shape.size() - 1]->xy;

                    beta *= 1.0/tvmet::norm2(beta);

                    float angle = acos(dot(alpha, beta));

                    if (angle < min_angle)
                    {
                        min_angle = angle;
                    }
                }

                double len_thus_far = 0;
                double _len = 0;
                //Remove elements until the next segment's length is greater than offset - previous segments.
                int new_start = -1;
                do{
                    //TODO could go infinite for tiny roads.
                    len_thus_far += _len;
                    new_start++;
                    vec2d start = e.shape[new_start + 1]->xy;
                    start -= e.shape[new_start]->xy;
                    _len = sqrt(start[0]*start[0] + start[1]*start[1]);
                }while (len_thus_far + _len<= tmp_offset); //TODO degenerate case when equal.

                //Modify geometry to make room for intersection.
                vec2d start_seg = e.shape[new_start + 1]->xy;
                start_seg -= e.shape[new_start]->xy;

                double len = sqrt(start_seg[0]*start_seg[0] + start_seg[1]*start_seg[1]);
                double factor = (len - (tmp_offset - len_thus_far))/len;
                start_seg *= factor;

                e.shape[new_start] = new node(*e.shape[new_start]);
                e.shape[new_start]->xy[0] = e.shape[new_start + 1]->xy[0] - start_seg[0];
                e.shape[new_start]->xy[1] = e.shape[new_start + 1]->xy[1] - start_seg[1];

                if (new_start != 0)
                {
                    e.shape.erase(e.shape.begin(), e.shape.begin() + new_start);
                }
            }

            BOOST_FOREACH(edge* edge_p, i.edges_ending_here)
            {
                edge& e = (*edge_p);

                double len_thus_far = 0;
                double _len = 0;
                int new_end = e.shape.size();
                do{
                    //TODO could go infinite for tiny roads.
                    len_thus_far += _len;
                    new_end--;
                    vec2d seg = e.shape[new_end]->xy;
                    seg -= e.shape[new_end - 1]->xy;
                    _len = sqrt(seg[0]*seg[0] + seg[1]*seg[1]);
                }while(len_thus_far + _len <= tmp_offset);

                int size = e.shape.size();
                vec2d end_seg = e.shape[new_end]->xy;

                end_seg -=  e.shape[new_end - 1]->xy;

                double len = sqrt(end_seg[0]*end_seg[0] + end_seg[1]*end_seg[1]);
                double factor = (len - (tmp_offset - len_thus_far))/len;

                end_seg *= factor;

                e.shape[new_end] = new node(*e.shape[new_end]);

                e.shape[new_end]->xy[0] = e.shape[new_end - 1]->xy[0] + end_seg[0];

                e.shape[new_end]->xy[1] = e.shape[new_end - 1]->xy[1] + end_seg[1];

                if (new_end != e.shape.size() - 1)
                {
                    e.shape.erase(e.shape.begin() + new_end + 1, e.shape.end());
                }
            }
        }
        return true;
    }

    bool network::scale_and_translate()
    {
        bool first = true;
        vec2d bias;

        BOOST_FOREACH(osm::node_pair &np, nodes)
        {
            if (first)
            {
                vec2d pt = np.second.xy;
                first = false;
                bias = pt*scale;
            }


            np.second.xy[0] = np.second.xy[0]*scale - bias[0];
            np.second.xy[1] = np.second.xy[1]*scale - bias[1];
        }
    }

    bool network::join(osm::edge* a, osm::edge* b)
    {
        assert(a->shape[0]->id == a->from);
        assert(a->shape[a->shape.size() - 1]->id == a->to);
        assert(b->shape[0]->id == b->from);
        assert(b->shape[b->shape.size() - 1]->id == b->to);
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

    bool edge::reverse()
    {
        std::swap(from, to);
        std::reverse(shape.begin(), shape.end());
    }

    bool network::join_logical_roads()
    {
        for(int i = 0; i < edges.size(); i++)
        {
            edge& e = edges[i];
            assert(e.to == e.shape.back()->id);
            assert(e.from == e.shape[0]->id);
            for (int j = 0; j < edges.size(); )
            {
                if (i == j)
                {
                    j++;
                    continue;
                }

                edge& o = edges[j];

                if ((nodes[e.to].id == nodes[o.from].id) and (node_degrees[e.to] == 2)){
                    node_degrees[e.to]--;

                    int e_size = e.shape.size();
                    int o_size = o.shape.size();
                    join(&e, &o);
                    assert(e.shape.size() == e_size + o_size - 1);


                    std::swap(edges[j], edges[edges.size() - 1]);
                    //Don't invalidate e if e is at the end.
                    if (i == edges.size() - 1)
                        e = edges[j];
                    edges.pop_back();
                }
                else if ((nodes[e.to].id == nodes[o.to].id) and (node_degrees[e.to] == 2)){
                    node_degrees[e.to]--;

                    int e_size = e.shape.size();
                    int o_size = o.shape.size();

                    o.reverse();
                    join(&e, &o);

                    assert(e.shape.size() == e_size + o_size - 1);


                    std::swap(edges[j], edges[edges.size() - 1]);
                    //Don't invalidate e if e is at the end.
                    if (i == edges.size() - 1)
                        e = edges[j];
                    edges.pop_back();
                }
                else if ((nodes[e.from].id == nodes[o.from].id) and (node_degrees[e.from] == 2)){
                    node_degrees[e.from]--;

                    int e_size = e.shape.size();
                    int o_size = o.shape.size();

                    e.reverse();
                    join(&e, &o);

                    assert(e.shape.size() == e_size + o_size - 1);

                    std::swap(edges[j], edges[edges.size() - 1]);
                    //Don't invalidate e if e is at the end.
                    if (i == edges.size() - 1)
                        e = edges[j];
                    edges.pop_back();
                }
                else
                {
                    j++;
                }
            }
        }
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
        BOOST_FOREACH(edge &ep, edges)
        {
            BOOST_FOREACH(node *node, ep.shape)
             {

                //Skip the first and last nodes.
                if (ep.from == node->id)
                    continue;

                if (ep.to == node->id)
                    continue;

                if (node_degrees[node->id] > 1)
                {
                    road_split_points[ep.id].push_back(node->id);
                }
            }
        }

        //Split each edge at its split points.
        std::vector<edge> new_edges;
        BOOST_FOREACH(edge &_edge, edges)
        {
            int node_index = 0;
            int _edge_ending_node = 0;

            //Find first splitter.
            bool _first = true;
            BOOST_FOREACH(str id, road_split_points[_edge.id])
            {

                //Skip the first splitter if it's the first point of the road.
                if (id == _edge.shape[0]->id){
                    continue;
                }

                //Increase the node degree as the road is being split.
                node_degrees[id]++;

                if (not _first)
                 {
                    new_edges.push_back(copy_no_shape(_edge));
                    (--new_edges.end())->from = _edge.shape[node_index]->id;
                    (--new_edges.end())->shape.push_back(&nodes[_edge.shape[node_index]->id]);
                }

                //Add each node to new edge up to, and including, the next splitter
                //Set the first node of the new edge

                while (_edge.shape[node_index]->id != id)
                {
                    node_index++;
                    if (not _first)
                    {
                        //Add nodes to new road
                        (--new_edges.end())->shape.push_back(&nodes[_edge.shape[node_index]->id]);
                        (--new_edges.end())->to = _edge.shape[node_index]->id;
                    }
                }

                if (not _first)
                    assert((--new_edges.end())->shape.size() > 1);

                //Node index is at the index of the split point.
                if (_first)
                {
                    //Update the node this road goes up to.
                    _edge.to = _edge.shape[node_index]->id;

                    _edge_ending_node = node_index;

                    _first = false;
                }

            }

            if (road_split_points[_edge.id].size() > 0)
            {
                //Now node_index is on the final splitter.
                //Add that splitter and all remaining nodes to a new edge.
                new_edges.push_back(copy_no_shape(_edge));
                (--new_edges.end())->from = _edge.shape[node_index]->id;
                for (;node_index < _edge.shape.size(); node_index++){
                    (--new_edges.end())->shape.push_back(_edge.shape[node_index]);
                    (--new_edges.end())->to = _edge.shape[node_index]->id;
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
            edges.push_back(e);
        }

        return true;
    }


    bool network::compute_edge_types()
    {
        BOOST_FOREACH(edge &e, edges)
        {
            edge_type* e_type = retrieve<edge_type>(types, e.id);
            e.type = e_type;
            e_type->speed = 25;
            e_type->nolanes = 1;
            e_type->oneway = 0;
            if(e.highway_class == "motorway")
            {
                e_type->speed = 65;
                e_type->nolanes = 2;
                e_type->oneway = 1;
            }
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

            //Classes added for grid
            if(e.highway_class == "urban")
            {
                e_type->speed = 30;
                e_type->nolanes = 2;
                e_type->oneway = 0;
            }
        }

        BOOST_FOREACH(osm::edge &e, edges)
        {
            assert(e.type->nolanes > 0);
            assert(e.type->nolanes < 10);
        }


    }
}
