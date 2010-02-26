#include "osm_network.hpp"
#include "arc_road.hpp"
#include <vector>
#include <sstream>
#include <limits>
#include <algorithm>

double scale = 157253.2964;

namespace osm
{
    //Instantiate static
    size_t network::new_edges_id = 0;

    typedef std::pair<const str, edge>         edge_pair;
    typedef std::pair<const str, node>         node_pair;
    typedef std::pair<const str, intersection> intr_pair;

    bool network::out_of_bounds(const vec3f &pt) const
    {
        return !((pt[0] >= topleft[0] and pt[0] <= bottomright[0])
                 and
                 (pt[1] >= bottomright[1] and pt[1] <= topleft[1]));
    }

    void network::clip_roads_to_bounds()
    {
        //Remove any node that's outside the bounding box.
        int j = 0;
        while (j < edges.size())
        {
            edge& e = edges[j];
            int   i = 0;
            while (i < e.shape.size())
                if (out_of_bounds(e.shape[i]->xy))
                    e.shape.erase(e.shape.begin() + i);
                else
                    i++;

            // e.shape.erase(e.shape.begin(), e.shape.begin() + new_start);
            // e.shape.erase(e.shape.begin() + new_end, e.shape.end());
            e.to   = e.shape.back()->id;
            e.from = e.shape[0]->id;

            if (e.shape.size() < 2)
                edges.erase(edges.begin() + j);
            else
                j++;
        }
    }

    void network::populate_edges_from_hash()
    {
        BOOST_FOREACH(strhash<edge>::type::value_type& hash, edge_hash)
        {
            edges.push_back(hash.second);
        }
    }

    void network::node_degrees_and_edges_agree()
    {
        strhash<int>::type node_degree_check;
        BOOST_FOREACH(const osm::node_pair &np, nodes)
        {
            node_degree_check.insert(std::pair<str,int>(np.first, 0));
        }

        BOOST_FOREACH(osm::edge &e, edges)
        {
            BOOST_FOREACH(osm::node *n, e.shape)
            {
                node_degree_check[n->id]++;
            }
        }

        typedef std::pair<const str, int> n_d;
        BOOST_FOREACH(n_d& nodepair, node_degree_check)
        {
            assert(node_degrees[nodepair.first] == nodepair.second);
        }

        //Check intersections
        BOOST_FOREACH(intr_pair& ip, intersections)
        {
            assert(node_degrees[ip.first] > 1);
        }

        BOOST_FOREACH(const edge& ep, edges)
        {
            assert(ep.to == ep.shape.back()->id);
            assert(ep.from == ep.shape[0]->id);
        }
    }

    void network::populate_edge_hash_from_edges()
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
        float _len         = 0;
        for(int i = 0; i < static_cast<int>(shape.size()) - 1; i++)
        {
            vec3f start   = shape[i + 1]->xy;
            start        -= shape[i]->xy;
            _len          = sqrt(start[0]*start[0] + start[1]*start[1]);
            len_thus_far += _len;
        }

        return len_thus_far;
    }

    void network::remove_small_roads(double min_len)
    {
        for(int i = 0; i < static_cast<int>(edges.size()); )
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

                //Update degree count for all nodes
                BOOST_FOREACH(node* n, e.shape)
                {
                    node_degrees[n->id]--;
                }

                //TODO join roads that this edge connected..
                std::swap(edges[i], edges.back());
                edges.pop_back();
            }
            else
                ++i;
        }
    }

    void network::create_grid(int w, int h, double dw, double dh)
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
                    str   e_id = n->id+"to"+node_grid[i][j-1]->id;
                    edge* e    = NULL;
                    for(int k=0; k < static_cast<int>(edges.size()); k++)
                        if (edges[k].id == e_id)
                        {
                            e = &edges[k];
                            break;
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
                    str   e_id = n->id+"to"+node_grid[i-1][j]->id;
                    edge* e    = NULL;
                    for(int k=0; k < static_cast<int>(edges.size()); k++)
                        if (edges[k].id == e_id)
                        {
                            e = &edges[k];
                            break;
                        }
                    if (e == NULL)
                    {
                        edges.push_back(edge());
                        e = &edges.back();
                    }
                    e->id            = e_id;
                    e->from          = n->id;
                    e->to            = node_grid[i-1][j]->id;
                    e->highway_class = "urban";
                    e->shape.push_back(&nodes[e->from]);
                    e->shape.push_back(&nodes[e->to]);
                }
            }
        }
    }

    void network::compute_node_degrees()
    {
        node_degrees.erase(node_degrees.begin(), node_degrees.end());

        BOOST_FOREACH(const osm::node_pair &np, nodes)
        {
            node_degrees.insert(std::pair<str,int>(np.first, 0));
        }

        BOOST_FOREACH(osm::edge &e, edges)
        {
            BOOST_FOREACH(osm::node *n, e.shape)
            {
                node_degrees[n->id]++;
                n->edges_including.push_back(&e);
            }
        }
    }

    void network::edges_including_rebuild()
    {
        BOOST_FOREACH(osm::node_pair &np, nodes)
        {
            np.second.edges_including.erase(np.second.edges_including.begin(), np.second.edges_including.end());
        }

        BOOST_FOREACH(osm::edge &e, edges)
        {
            BOOST_FOREACH(node* n, e.shape)
            {
                if (find(n->edges_including.begin(), n->edges_including.end(), &e) == n->edges_including.end())
                    n->edges_including.push_back(&e);
            }
        }
    }

    void network::create_ramps()
    {
        edges_including_rebuild();

        BOOST_FOREACH(osm::edge &e, edges)
        {
            if (e.highway_class == "motorway_link")
            {
                for (int i = 0; i < e.shape.size(); i++)
                {
                    node*& n = e.shape[i];
                    if (n->ramp_merging_point != NULL)
                    {
                        node*      highway_node         = n->ramp_merging_point;
                        bool       highway_intersection = false;
                        osm::edge* highway              = NULL;
                        BOOST_FOREACH(osm::edge *e, highway_node->edges_including)
                        {
                            highway_intersection = (e->highway_class == "motorway") or highway_intersection;
                            if (highway_intersection)
                                highway = e;
                        }

                        if (highway_intersection)
                        {
                            //Create arc road for highway.
                            arc_road highway_shape;
                            BOOST_FOREACH(node* foo, highway->shape)
                            {
                                highway_shape.points_.push_back(foo->xy);
                            }

                            highway_shape.initialize_from_polyline(0.7, highway_shape.points_);

                            //Find index of intersection point
                            int index = 0;
                            for (index = 0;
                                 index < highway_shape.points_.size();
                                 index++)
                            {
                                if (tvmet::all_elements(highway_shape.points_[index] == highway_node->xy))
                                {
                                    std::cout << "Found at index " << index << std::endl;
                                    break;
                                }
                            }

                            //Find t of point.

                            ///First, we need the offset of the point where the ramp will merge.
                            ///This will always be on the right side of the road, one lane beyond the end of the road.
                            float lanect  = -1;
                            float nolanes = 2;

                            //TODO use lane width value, not constant
                            float offset = (2.5)*(lanect + -((nolanes - 1)/2.0));

                            ///length up to feature i
                            ///+ 1/2 length of feature i
                            //// length of total road
                            int feature_index = (2*(index + 1)) - 3;
                            if (index == 0){ feature_index = 0;}
                            if (index == highway_shape.points_.size() - 1){ feature_index--;}

                            float t = (highway_shape.feature_base(feature_index, offset) + (highway_shape.feature_size(feature_index, offset) / 2.0)) / highway_shape.length(offset);

                            std::cout << t << std::endl;

                            vec3f pt = highway_shape.point(t, offset);

                            n->xy = pt;

                            float len = 15;
                            //Load the tangent based on the direction of the ramp.
                            //Move the next to last (or second) point to make the ramp tangent to the highway.
                            if (i == 0)
                            {
                                vec3f tan(col(highway_shape.frame(t, offset, false), 0));
                                e.shape[i + 1]->xy = len*tan + n->xy;
                            }
                            else
                            {
                                vec3f tan(col(highway_shape.frame(t, offset, true), 0));
                                e.shape[i - 1]->xy = len*tan + n->xy;
                            }

                            float t_center                  = (highway_shape.feature_base(feature_index, 0.0) + (highway_shape.feature_size(feature_index, 0.0) / 2.0)) / highway_shape.length(0.0);
                            float merge_lane_len            = 60;
                            float merge_lane_parametric_len = merge_lane_len / highway_shape.length(0.0);

                            //Add a merging lane to the highway
                            if (i == 0)//Offramp
                                highway->additional_lanes.push_back(osm::edge::lane(t_center - merge_lane_parametric_len,t_center,offset,e.id, true));
                            else//Onramp
                                highway->additional_lanes.push_back(osm::edge::lane(t_center,t_center + merge_lane_parametric_len,offset,e.id, false));

                        }
                    }
                }
            }
        }
    }

    void network::remove_highway_intersections()
    {
        edges_including_rebuild();

        BOOST_FOREACH(osm::edge &e, edges)
        {
            if (e.highway_class == "motorway")
            {
                for(int i = 0; i < static_cast<int>(e.shape.size()); i++)
                {
                    bool   ramp_node = false;
                    node*& n         = e.shape[i];

                    if (node_degrees[n->id] > 1)
                    {
                        //Removing node from highway
                        node_degrees[n->id]--;

                        node* old    = n;
                        str   new_id = old->id + "_HWY";
                        n            = retrieve<node>(nodes, new_id);

                        //If there is a ramp at this intersection, store the connecting node
                        BOOST_FOREACH(edge* e, old->edges_including)
                        {
                            if (e->highway_class == "motorway_link")
                            {
                                old->ramp_merging_point = n;
                                ramp_node = true;
                            }
                        }

                        //Make old node an overpass.
                        if (!ramp_node)
                            old->is_overpass = true;

                        n->xy = old->xy;
                        //TODO edges_including..
                        n->id = new_id;
                        n->edges_including.push_back(&e);
                        if (find(old->edges_including.begin(),
                                 old->edges_including.end(),
                                 &e) != old->edges_including.end())
                            old->edges_including.erase(find(old->edges_including.begin(),
                                                            old->edges_including.end(),
                                                            &e));

                        if (node_degrees.find(n->id) == node_degrees.end())
                            node_degrees[n->id] = 0;
                        node_degrees[n->id]++;

                        //If node is at the end of the road
                        if (i == static_cast<int>(e.shape.size()) - 1)
                            e.to = n->id;
                        if (i == 0)
                            e.from = n->id;
                    }
                }
            }
        }
    }

    typedef std::pair<vec2f, vec2f> pair_of_isects;
    static pair_of_isects circle_line_intersection(const vec2f &pt1,
                                                   const vec2f &pt2,
                                                   const vec2f &center,
                                                   float r)
    {
        const float x1 = pt1[0] - center[0];
        const float y1 = pt1[1] - center[1];
        const float x2 = pt2[0] - center[0];
        const float y2 = pt2[1] - center[1];

        const float dx = (x2 - x1);
        const float dy = (y2 - y1);
        const float dr = sqrt(pow(dx,2) + pow(dy,2));
        const float D  = x1*y2 - x2*y1;

        const vec2f isect1((D*dy + copysign(1, dy)*dx*sqrt(r*r*dr*dr - D*D))/(dr*dr),
                           -D*dx + std::abs(dy)*sqrt(r*r*dr*dr - D*D)/(dr*dr));

        const vec2f isect2((D*dy - copysign(1, dy)*dx*sqrt(r*r*dr*dr - D*D))/(dr*dr),
                           -D*dx - std::abs(dy)*sqrt(r*r*dr*dr - D*D)/(dr*dr));

        return std::make_pair(isect1 + center, isect2 + center);
    }

    void network::compute_node_heights()
    {
        float overpass_height = 15;

        BOOST_FOREACH(osm::edge& e, edges)
        {
            for(int i = 0; i < static_cast<int>(e.shape.size()); i++)
            {
                osm::node* n = e.shape[i];
                //Find node that's part of overpass.
                if (n->is_overpass)
                {
                    //TODO What if road ends before ramp radius is reached? Could continue to traverse connecting roads, but that could produce odd effects.

                    n->xy[2] = overpass_height;

                    vec2f n_2d(n->xy[0], n->xy[1]);

                    //Walk back until
                    //  1) another overpass is found, or
                    //  2) until a line segment intersects with a circle centered at point i
                    // for(int j = i - 1; j >= 0; j--)
                    // {
                    //     osm::node* j_node = e.shape[j];

                    //     vec2f j_2d(j_node->xy[0], j_node->xy[1]);

                    //     if (j_node->is_overpass)
                    //         break;

                    //     float dist = norm2(j_2d - n_2d);
                    //     if (dist > overpass_radius)
                    //     {
                    //         //Intersect line segment (j and j + 1) with circle and add point.
                    //         vec2f jp1_2d(e.shape[j + 1]->xy[0], e.shape[j + 1]->xy[1]);

                    //         std::pair<vec2f, vec2f> isects =
                    //             circle_line_intersection(j_2d, jp1_2d, n_2d, overpass_radius);

                    //         bool first_is_on_seg = ((std::min(j_2d[0], jp1_2d[0]) < isects.first[0])
                    //                                 and
                    //                                 (std::max(j_2d[0], jp1_2d[0]) > isects.first[0])
                    //                                 and
                    //                                 (std::min(j_2d[1], jp1_2d[1]) < isects.first[1])
                    //                                 and
                    //                                 (std::max(j_2d[1], jp1_2d[1]) > isects.first[1]));

                    //         bool second_is_on_seg = ((std::min(j_2d[0], jp1_2d[0]) < isects.second[0])
                    //                                 and
                    //                                 (std::max(j_2d[0], jp1_2d[0]) > isects.second[0])
                    //                                 and
                    //                                 (std::min(j_2d[1], jp1_2d[1]) < isects.second[1])
                    //                                 and
                    //                                 (std::max(j_2d[1], jp1_2d[1]) > isects.second[1]));

                    //         if (first_is_on_seg)
                    //         {
                    //             //Add first intersection as new node on the line.
                    //             osm::node ramp_start;
                    //             ramp_start.id = n->id + "_opassramp";
                    //             ramp_start.xy = vec3f(isects.first[0], isects.first[1], 0.0f);
                    //             ramp_start.edges_including.push_back(&e);
                    //             node_degrees[ramp_start.id] = 1;
                    //             nodes.insert(std::make_pair(ramp_start.id, ramp_start));
                    //         }
                    //         else if (second_is_on_seg)
                    //         {
                    //             //Add second intersection as new node on the line.
                    //         }
                    //         else if (first_is_on_seg and second_is_on_seg)
                    //         {
                    //             assert(0);
                    //         }
                    //         else
                    //         {
                    //             assert(0);
                    //         }


                    //    break;
                    // }
                    //}
                }
            }
        }
    }

    void network::intersection_check()
    {
        BOOST_FOREACH(const osm::intr_pair &ip, intersections)
        {
            assert(ip.first == ip.second.id_from_node);
            assert(node_degrees[ip.first] > 1);
        }

        //Check that intersections don't occur in the middle of roads.
        BOOST_FOREACH(const osm::edge &e, edges)
        {
            for (int i = 1; i < static_cast<int>(e.shape.size()) - 1; i++)
                assert(node_degrees[e.shape[i]->id] == 1);
        }
    }

    void network::create_intersections()
    {
        compute_node_degrees();

        BOOST_FOREACH(osm::edge &e, edges)
        {
            assert(e.to == e.shape.back()->id);
            assert(e.from == e.shape[0]->id);
            if (node_degrees[e.to] > 1)
            {
                if (e.highway_class == "motorway")
                {
                    std::cout << e.id << " is an intersection to " << e.to << std::endl;
                    std::cout << node_degrees[e.to] << std::endl;
                }
                intersection* curr;
                curr = retrieve<intersection>(intersections, e.to);
                curr->edges_ending_here.push_back(&e);
                curr->id_from_node = e.to;
            }
            if (node_degrees[e.from] > 1)
            {
                if (e.highway_class == "motorway")
                {
                    std::cout << e.id << " is an intersection from " << e.from << std::endl;
                    std::cout << node_degrees[e.from] << std::endl;
                }
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
                // vec3f alpha(e.shape[1]->xy[0] - e.shape[0]->xy[0],
                //                               e.shape[1]->xy[1] - e.shape[0]->xy[1],
                //                               0.0f);
                // alpha *= 1.0/tvmet::norm2(alpha);

                // double min_angle = std::numeric_limits<double>::infinity();
                // BOOST_FOREACH(edge* other_edge_p, i.edges_starting_here)
                // {
                //     edge& f = (*other_edge_p);

                //     if (e.id == f.id)
                //         continue;

                //     vec3f beta(f.shape[1]->xy[0] - f.shape[0]->xy[0],
                //                                  f.shape[1]->xy[1] - f.shape[0]->xy[1],
                //                                  0.0f);
                //     beta *= 1.0/tvmet::norm2(beta);

                //     float angle = acos(dot(alpha, beta));

                //     if (angle < min_angle)
                //     {
                //         min_angle = angle;
                //     }
                // }

                // BOOST_FOREACH(edge* other_edge_p, i.edges_ending_here)
                // {
                //     edge& f = (*other_edge_p);

                //     vec3f beta;
                //     beta = f.shape[f.shape.size() - 2]->xy;
                //     beta -= f.shape[f.shape.size() - 1]->xy;

                //     beta *= 1.0/tvmet::norm2(beta);

                //     float angle = acos(dot(alpha, beta));

                //     if (angle < min_angle)
                //     {
                //         min_angle = angle;
                //     }
                // }

                double len_thus_far = 0;
                double _len         = 0;
                //Remove elements until the next segment's length is greater than offset - previous segments.
                int    new_start    = -1;
                do
                {
                    //TODO could go infinite for tiny roads.
                    len_thus_far                 += _len;
                    new_start++;
                    tvmet::Vector<float,3> start  = e.shape[new_start + 1]->xy;
                    start                        -= e.shape[new_start]->xy;
                    _len                          = sqrt(start[0]*start[0] + start[1]*start[1]);
                }
                while (len_thus_far + _len<= tmp_offset); //TODO degenerate case when equal.

                //Update node degree count.
                for (int i = 1; i <= new_start; i++)
                    node_degrees[e.shape[i]->id]--;

                //Modify geometry to make room for intersection.
                vec3f start_seg = e.shape[new_start + 1]->xy;
                start_seg -= e.shape[new_start]->xy;

                double len = sqrt(start_seg[0]*start_seg[0] + start_seg[1]*start_seg[1]);
                double factor = (len - (tmp_offset - len_thus_far))/len;
                start_seg *= factor;

                e.shape[new_start]        = new node(*e.shape[new_start]);
                e.shape[new_start]->id    = e.shape[0]->id;
                e.shape[new_start]->xy[0] = e.shape[new_start + 1]->xy[0] - start_seg[0];
                e.shape[new_start]->xy[1] = e.shape[new_start + 1]->xy[1] - start_seg[1];

                if (new_start != 0)
                    e.shape.erase(e.shape.begin(), e.shape.begin() + new_start);
            }

            BOOST_FOREACH(edge* edge_p, i.edges_ending_here)
            {
                edge& e = (*edge_p);

                double len_thus_far = 0;
                double _len         = 0;
                int    new_end      = e.shape.size();
                do
                {
                    //TODO could go infinite for tiny roads.
                    len_thus_far += _len;
                    new_end--;
                    vec3f seg     = e.shape[new_end]->xy;
                    seg          -= e.shape[new_end - 1]->xy;
                    _len          = sqrt(seg[0]*seg[0] + seg[1]*seg[1]);
                }
                while(len_thus_far + _len <= tmp_offset);

                //Update node degree count
                //Don't change count for the last node, as we use its id.
                for (int i = new_end; i < static_cast<int>(e.shape.size()) - 1; i++)
                    node_degrees[e.shape[i]->id]--;

                vec3f end_seg = e.shape[new_end]->xy;

                end_seg -=  e.shape[new_end - 1]->xy;

                double len    = sqrt(end_seg[0]*end_seg[0] + end_seg[1]*end_seg[1]);
                double factor = (len - (tmp_offset - len_thus_far))/len;

                end_seg                 *= factor;
                e.shape[new_end]         = new node(*e.shape[new_end]);
                e.shape[new_end]->id     = e.shape[e.shape.size() - 1]->id;
                e.shape[new_end]->xy[0]  = e.shape[new_end - 1]->xy[0] + end_seg[0];
                e.shape[new_end]->xy[1]  = e.shape[new_end - 1]->xy[1] + end_seg[1];

                if (new_end != static_cast<int>(e.shape.size()) - 1)
                    e.shape.erase(e.shape.begin() + new_end + 1, e.shape.end());
            }
        }
    }

    void network::scale_and_translate()
    {
        bool first = true;
        vec2d bias;

        BOOST_FOREACH(osm::node_pair &np, nodes)
        {
            if (first)
                bias = center*scale;
            np.second.xy[0] = np.second.xy[0]*scale - bias[0];
            np.second.xy[1] = np.second.xy[1]*scale - bias[1];
        }
    }

    void network::join(osm::edge* a, osm::edge* b)
    {
        //Adds b to a
        assert(a->shape[0]->id == a->from);
        assert(a->shape[a->shape.size() - 1]->id == a->to);
        assert(b->shape[0]->id == b->from);
        assert(b->shape[b->shape.size() - 1]->id == b->to);
        assert(a->to == b->from);

        uint a_size = a->shape.size();
        uint b_size = b->shape.size();

        //Add all of b's nodes except the first one.
        for (int i = 1; i < static_cast<int>(b->shape.size()); i++)
            a->shape.push_back(b->shape[i]);

        a->to = b->to;

        assert(a->shape.size() == a_size + b_size - 1);
    }

    void edge::reverse()
    {
        std::swap(from, to);
        std::reverse(shape.begin(), shape.end());
    }

    void network::join_logical_roads()
    {
        for(int i = 0; i < static_cast<int>(edges.size()); i++)
        {
            edge& e = edges[i];
            assert(e.to == e.shape.back()->id);
            assert(e.from == e.shape[0]->id);
            for (int j = 0; j < static_cast<int>(edges.size()); )
            {
                if (i == j)
                {
                    j++;
                    continue;
                }

                edge& o = edges[j];
                //Combine roads at degenerate intersections.
                if ((nodes[e.to].id == nodes[o.from].id) and (node_degrees[e.to] == 2))
                {
                    node_degrees[e.to]--;

                    int e_size = e.shape.size();
                    int o_size = o.shape.size();
                    join(&e, &o);
                    assert(static_cast<int>(e.shape.size()) == e_size + o_size - 1);

                    std::swap(edges[j], edges[edges.size() - 1]);
                    //Don't invalidate e if e is at the end.
                    if (i == static_cast<int>(edges.size()) - 1)
                        e = edges[j];
                    edges.pop_back();
                }
                else if ((nodes[e.to].id == nodes[o.to].id) and (node_degrees[e.to] == 2))
                {
                    node_degrees[e.to]--;

                    int e_size = e.shape.size();
                    int o_size = o.shape.size();

                    o.reverse();
                    join(&e, &o);

                    assert(static_cast<int>(e.shape.size()) == e_size + o_size - 1);

                    std::swap(edges[j], edges[edges.size() - 1]);
                    //Don't invalidate e if e is at the end.
                    if (i == static_cast<int>(edges.size()) - 1)
                        e = edges[j];
                    edges.pop_back();
                }
                else if ((nodes[e.from].id == nodes[o.from].id) and (node_degrees[e.from] == 2))
                {
                    node_degrees[e.from]--;

                    int e_size = e.shape.size();
                    int o_size = o.shape.size();

                    e.reverse();
                    join(&e, &o);

                    assert(static_cast<int>(e.shape.size()) == e_size + o_size - 1);

                    std::swap(edges[j], edges[edges.size() - 1]);
                    //Don't invalidate e if e is at the end.
                    if (i == static_cast<int>(edges.size()) - 1)
                        e = edges[j];
                    edges.pop_back();
                }
                else
                    j++;
            }
        }
    }

    void network::display_used_node_heights()
    {
        BOOST_FOREACH(edge& e, edges)
        {
            BOOST_FOREACH(node* n, e.shape)
            {
                if (n->xy[2] > 0)
                    std::cout << n->id << " " << n->xy[2] << std::endl;
            }
        }
    }

    edge network::copy_no_shape(const edge& e)
    {
        edge to_return;
        to_return.type = e.type;

        //Initialized to values that must be changed.
        to_return.from = "-1";
        to_return.to   = "-1";

        std::stringstream sout;
        sout << network::new_edges_id;
        to_return.id = str(sout.str());
        network::new_edges_id++;

        to_return.highway_class = e.highway_class;

        return to_return;
    }

    void network::split_into_road_segments()
    {
        //Locate all split points.
        std::map<str, std::vector<str> > road_split_points;
        BOOST_FOREACH(edge &ep, edges)
        {
            int i = -1;
            BOOST_FOREACH(node *node, ep.shape)
            {
                i++;

                //Skip the first and last nodes.
                if (i == 0 or i == static_cast<int>(ep.shape.size()) - 1)
                    continue;

                if (node_degrees[node->id] > 1)
                    road_split_points[ep.id].push_back(node->id);
            }
        }

        //Split each edge at its split points.
        std::vector<edge> new_edges;
        BOOST_FOREACH(edge &_edge, edges)
        {
            int node_index        = 0;
            int _edge_ending_node = 0;

            //Find first splitter.
            bool _first = true;
            BOOST_FOREACH(str id, road_split_points[_edge.id])
            {
                //Skip the first splitter if it's the first point of the road.
                // if (id == _edge.shape[0]->id){
                //     assert(0);  //This should never occur.
                //     continue;
                // }

                if (not _first)
                {
                    new_edges.push_back(copy_no_shape(_edge));
                    new_edges.back().from = _edge.shape[node_index]->id;
                    new_edges.back().shape.push_back(&nodes[_edge.shape[node_index]->id]);
                }

                //Increase the node degree as the road is being split.
                node_degrees[id]++;

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

                        assert( (node_degrees[_edge.shape[node_index]->id] < 2) or _edge.shape[node_index]->id == id);
                    }
                }

                if (not _first)
                    assert((--new_edges.end())->shape.size() > 1);

                //Node index is at the index of the split point.
                if (_first)
                {
                    //Update the node this road goes up to.
                    _edge.to          = _edge.shape[node_index]->id;
                    _edge_ending_node = node_index;
                    _first            = false;
                }
            }

            if (road_split_points[_edge.id].size() > 0)
            {
                //Now node_index is on the final splitter.
                //Add that splitter and all remaining nodes to a new edge.
                new_edges.push_back(copy_no_shape(_edge));
                new_edges.back().from = _edge.shape[node_index]->id;

                for (;node_index < static_cast<int>(_edge.shape.size()); node_index++){
                    new_edges.back().shape.push_back(_edge.shape[node_index]);

                    new_edges.back().to = _edge.shape[node_index]->id;
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
    }

    void network::compute_edge_types()
    {
        BOOST_FOREACH(edge &e, edges)
        {
            edge_type* e_type = retrieve<edge_type>(types, e.id);
            e.type            = e_type;
            e_type->speed     = 25;
            e_type->nolanes   = 1;
            e_type->oneway    = 0;
            if(e.highway_class == "motorway")
            {
                e_type->speed   = 65;
                e_type->nolanes = 2;
                e_type->oneway  = 1;
            }
            if(e.highway_class == "motorway_link")
            {
                e_type->speed   = 30;
                e_type->nolanes = 1;
                e_type->oneway  = 1;
            }
            if(e.highway_class == "residential")
            {
                e_type->speed   = 30;
                e_type->nolanes = 1;
                e_type->oneway  = 0;
            }
            if(e.highway_class == "primary")
            {
                e_type->nolanes = 2;
                e_type->oneway  = 0;
                e_type->speed   = 50;
            }
            if(e.highway_class == "secondary")
            {
                e_type->speed   = 40;
                e_type->nolanes = 2;
                e_type->oneway  = 0;
            }
            if(e.highway_class == "service")
            {
                e_type->speed   = 25;
                e_type->nolanes = 1;
                e_type->oneway  = 0;
            }
            if(e.highway_class == "primary_link")
            {
                e_type->speed   = 30;
                e_type->nolanes = 1;
                e_type->oneway  = 1;
            }
            if(e.highway_class == "secondary_link")
            {
                e_type->speed   = 30;
                e_type->nolanes = 1;
                e_type->oneway  = 1;
            }
            //Classes added for grid
            if(e.highway_class == "urban")
            {
                e_type->speed   = 30;
                e_type->nolanes = 2;
                e_type->oneway  = 0;
            }
        }

        BOOST_FOREACH(osm::edge &e, edges)
        {
            assert(e.type->nolanes > 0);
            assert(e.type->nolanes < 10);
        }
    }
}
