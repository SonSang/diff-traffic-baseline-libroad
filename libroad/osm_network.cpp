#include "osm_network.hpp"
#include "arc_road.hpp"
#include <vector>
#include <sstream>
#include <limits>
#include <algorithm>

static const double moar_fudge   = 0.5; //0.6666666;
static const double scale        = 157253.2964 * moar_fudge;
static const float  MIPH_TO_MEPS = 1609.344/(60.0*60.0);

namespace osm
{
    //Instantiate static
    size_t network::new_edges_id = 0;

    typedef std::pair<const str, edge>         edge_pair;
    typedef std::pair<const str, node>         node_pair;
    typedef std::pair<const str, intersection> intr_pair;

    node *network::add_node(const vec3f &v, const bool is_overpass)
    {
        str id;
        strhash<node>::type::iterator res;
        do
        {
            id = boost::str(boost::format("%d") % rand());
            res = nodes.find(id);
        }
        while(res != nodes.end());

        res                     = nodes.insert(res, std::make_pair(id, node()));
        res->second.id          = id;
        res->second.xy          = v;
        res->second.is_overpass = is_overpass;

        return &(res->second);
    }

    bool network::out_of_bounds(const vec3f &pt) const
    {
        return !((pt[0] >= topleft[0] and pt[0] <= bottomright[0])
                 and
                 (pt[1] >= bottomright[1] and pt[1] <= topleft[1]));
    }

    void network::clip_roads_to_bounds()
    {
        //Remove any node that's outside the bounding box.
        size_t j = 0;
        while (j < edges.size())
        {
            edge&  e = edges[j];
            size_t i = 0;
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

    void network::remove_duplicate_nodes()
    {
        BOOST_FOREACH(edge& e, edges)
        {
            e.remove_duplicate_nodes();
        }
    }

    void network::edges_check()
    {
        BOOST_FOREACH(const edge& e, edges)
        {
            assert(e.shape.size() > 1);
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

    void network::list_edges()
    {
        BOOST_FOREACH(edge& e, edges)
        {
            std::cout << " edge id " << e.id << std::endl;
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
        for(int i = 0; i < static_cast<int>(shape.size()) - 1; i++)
            len_thus_far += planar_distance(shape[i + 1]->xy, shape[i]->xy);

        return len_thus_far;
    }

    void network::remove_small_roads(double min_len)
    {
        for(int i = 0; i < static_cast<int>(edges.size()); )
        {
            edge& e = edges[i];
            if (e.length() < min_len)
            {
                // //Update intersections
                // if (intersections.find(e.to) != intersections.end())
                // {
                //     intersection& i_to = intersections[e.to];
                //     i_to.edges_ending_here.erase(find(i_to.edges_ending_here.begin(), i_to.edges_ending_here.end(), &e));
                // }

                // if (intersections.find(e.from) != intersections.end())
                // {
                //     intersection& i_from = intersections[e.from];
                //     i_from.edges_starting_here.erase(find(i_from.edges_starting_here.begin(), i_from.edges_starting_here.end(), &e));
                // }

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
                n->xy[2] = 0.0;

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
        assert(node_degrees.size() == 0);
        BOOST_FOREACH(osm::node_pair &np, nodes)
        {
            np.second.edges_including.erase(np.second.edges_including.begin(), np.second.edges_including.end());
        }

        BOOST_FOREACH(const osm::node_pair &np, nodes)
        {
            node_degrees.insert(std::pair<str,int>(np.first, 0));
        }

        BOOST_FOREACH(osm::edge &e, edges)
        {
            BOOST_FOREACH(osm::node *n, e.shape)
            {
                //Check if this edge has already been recorded (edge has node multiple times
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
            assert(np.second.edges_including.size() == 0);
        }

        BOOST_FOREACH(osm::edge &e, edges)
        {
            BOOST_FOREACH(node* n, e.shape)
            {
                n->edges_including.push_back(&e);
            }
        }
    }


    void network::create_ramps(const float lane_width)
    {
        edges_including_rebuild();

        BOOST_FOREACH(osm::edge &e, edges)
        {
            //For each ramp..
            if (e.highway_class == "motorway_link")
            {
                for (int b = 0; b < 2; b++)
                {
                    size_t i;
                    if (b == 0)
                        i = 0;
                    else if (b == 1)
                        i = ((int)e.shape.size()) - 1;
                    else
                        assert(0);

                    node* n = e.shape[i];
                    if (n->ramp_merging_point != NULL)
                    {
                        node*      highway_node         = n->ramp_merging_point;
                        bool       highway_intersection = false;
                        osm::edge* highway              = NULL;
                        BOOST_FOREACH(osm::edge *e, highway_node->edges_including)
                        {
                            highway_intersection = (e->highway_class == "motorway");

                            if (highway_intersection)
                            {
                                highway = e;
                                break;
                            }
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
                            size_t index = 0;
                            float epsilon = 1e-10;
                            for (index = 0;
                                 index < highway_shape.points_.size();
                                 index++)
                            {
                                if (tvmet::all_elements(tvmet::abs(highway_shape.points_[index] - highway_node->xy) < epsilon))
                                {
                                    break;
                                }
                            }

                            if (index == highway_shape.points_.size())
                                continue;

                            //Find t of point.

                            ///First, we need the offset of the point where the ramp will merge.
                            ///This will always be on the right side of the road, one lane beyond the end of the road.
                            float lanect  = -1;
                            float nolanes = 2;

                            //TODO use lane width value, not constant
                            float offset = lane_width*(lanect + -((nolanes - 1)/2.0));

                            ///length up to feature i
                            ///+ 1/2 length of feature i
                            //// length of total road
                            int feature_index = (2*(index + 1)) - 3;
                            if (index == 0) feature_index = 0;
                            if (index == highway_shape.points_.size() - 1) feature_index--;

                            float t = (highway_shape.feature_base(feature_index, offset) + (highway_shape.feature_size(feature_index, offset) / 2.0)) / highway_shape.length(offset);

                            vec3f pt = highway_shape.point(t, offset);

                            n->xy = pt;

                            float len = 15;
                            //Load the tangent based on the direction of the ramp.
                            //Move the next to last (or second) point to make the ramp tangent to the highway.
                            if (i == 0)
                            {
                                vec3f tan(col(highway_shape.frame(t, offset, false), 0));
                                e.shape.insert(e.shape.begin() + 1, new node);
                                e.shape[i + 1]->id = str(boost::lexical_cast<std::string>(rand()));
                                e.shape[i + 1]->xy = len*tan + pt;
                            }
                            else if (i + 1 == e.shape.size())
                             {
                                vec3f tan(col(highway_shape.frame(t, offset, true), 0));
                                e.shape.insert(e.shape.begin() + i, new node);
                                e.shape[i]->id = str(boost::lexical_cast<std::string>(rand()));
                                e.shape[i]->xy = len*tan + pt;
                            }
                            else
                            {
                                assert(0); //The ramp should only connect at its edges.
                            }

                            float t_center                  = (highway_shape.feature_base(feature_index, 0.0) + (highway_shape.feature_size(feature_index, 0.0) / 2.0)) / highway_shape.length(0.0);
                            float merge_lane_len            = 60;
                            float merge_lane_parametric_len = merge_lane_len / highway_shape.length(0.0);

                            //Add a merging lane to the highway
                            if (i == 0)//Offramp
                            {
                                if (std::abs(std::max((float)0, t_center - merge_lane_parametric_len) - t_center) > 0)
                                {
                                    highway->additional_lanes.push_back(osm::edge::lane(std::max((float)0, t_center - merge_lane_parametric_len),
                                                                                    t_center,
                                                                                    offset,
                                                                                    e.id,
                                                                                    true));
                                }
                            }
                            else//Onramp
                            {
                                if (std::abs(std::min((float)1, t_center + merge_lane_parametric_len) - t_center) > 0)
                                {
                                    highway->additional_lanes.push_back(osm::edge::lane(t_center,
                                                                                    std::min((float)1, t_center + merge_lane_parametric_len),
                                                                                    offset,
                                                                                    e.id,
                                                                                    false));
                                }

                            }
                        }
                    }
                }
            }
        }
    }

    void network::remove_highway_intersections()
    {
        edges_including_rebuild();
        compute_node_degrees();

        BOOST_FOREACH(osm::edge &e, edges)
        {
            if (e.highway_class == "motorway")
            {
                for(size_t i = 0; i < e.shape.size(); i++)
                {
                    bool   ramp_node = false;
                    node*& n         = e.shape[i];

                    //Found an intersection
                    if (node_degrees[n->id] > 2)
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
                        if (i + 1 == e.shape.size())
                            e.to = n->id;
                        if (i == 0)
                            e.from = n->id;

                    }
                    else if (n->ramp_merging_point != NULL or n->is_overpass == true)
                    {
                        //Removing node from highway
                        node_degrees[n->id]--;

                        node* old    = n;
                        str   new_id = old->id + "_HWY";
                        n            = retrieve<node>(nodes, new_id);
                        n->xy = old->xy;
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
                        if (i + 1 == e.shape.size())
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
                                                   float r)  __attribute__ ((unused));

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
        const float dr = std::sqrt(std::pow(dx,2) + std::pow(dy,2));
        const float D  = x1*y2 - x2*y1;

        const vec2f isect1((D*dy + copysign(1, dy)*dx*std::sqrt(r*r*dr*dr - D*D))/(dr*dr),
                           -D*dx + std::abs(dy)*std::sqrt(r*r*dr*dr - D*D)/(dr*dr));

        const vec2f isect2((D*dy - copysign(1, dy)*dx*std::sqrt(r*r*dr*dr - D*D))/(dr*dr),
                           -D*dx - std::abs(dy)*std::sqrt(r*r*dr*dr - D*D)/(dr*dr));

        return std::make_pair(isect1 + center, isect2 + center);
    }

    void network::compute_node_heights()
    {
        float overpass_height = 5;

        BOOST_FOREACH(osm::edge& e, edges)
        {
            std::vector<node*> new_shape;
            for(int i = 0; i < static_cast<int>(e.shape.size()); i++)
            {
                osm::node* n = e.shape[i];
                //Find node that's part of overpass.
                if (n->is_overpass)
                {
                    //TODO What if road ends before ramp radius is reached? Could continue to traverse connecting roads, but that could produce odd effects.

                    n->xy[2] += overpass_height;

            //         vec2f n_2d(n->xy[0], n->xy[1]);

            //         //Walk back until
            //         //  1) another overpass is found, or
            //         //  2) until a line segment intersects with a circle centered at point i
            //         float overpass_radius = 60;
            //         float len_thus_far = 0;
            //         int last_index = -1;
            //         for(int j = i - 1; j >= 0; j--)
            //         {
            //             osm::node* j_node = e.shape[j];

            //             vec3f j_3d(j_node->xy[0], j_node->xy[1], 0);
            //             vec3f jp1_3d(e.shape[j + 1]->xy[0], e.shape[j + 1]->xy[1], 0);

            //             if (j_node->is_overpass)
            //                 break;

            //             float dist = norm2(j_3d - jp1_3d);
            //             if (len_thus_far + dist > overpass_radius)
            //             {
            //                 float param = (overpass_radius - len_thus_far) / dist;
            //                 vec3f new_point(param*(j_3d - jp1_3d) + jp1_3d);
            //                 osm::node ramp_start;
            //                 ramp_start.id = n->id + "_opassramp";
            //                 ramp_start.xy = vec3f(new_point);
            //                 ramp_start.edges_including.push_back(&e);
            //                 node_degrees[ramp_start.id] = 1;
            //                 strhash<node>::type::iterator pos = nodes.insert(std::make_pair(ramp_start.id, ramp_start)).first;
            //                 new_shape.push_back(&(pos->second));
            //                 break;
            //             }
            //             else
            //             {
            //                 len_thus_far += dist;
            //                 new_shape[j]->xy[2] = (len_thus_far / overpass_radius) / overpass_height;
            //             }
            //         }
            //         new_shape.push_back(n);
            //     }
            //     else
            //         new_shape.push_back(n);
                }

            // e.shape.clear();
            // for (int i = 0; i < new_shape.size(); i++)
            // {
            //     e.shape.push_back(new_shape[i]);
            // }
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

    namespace create_intersections
    {
        struct Edge_Offset
        {
            float offset;
            vec2f approach_vector;
            int num_of_lanes;
            float angle;
            osm::edge* parent_edge;
            static bool clockwise(const Edge_Offset& a, const Edge_Offset& b){return a.angle < b.angle;}
        };
    };

    void network::create_intersections(float lane_width)
    {
        compute_node_degrees();

        BOOST_FOREACH(osm::edge &e, edges)
        {
            assert(e.to == e.shape.back()->id);
            assert(e.from == e.shape[0]->id);
            if (node_degrees[e.to] > 2)
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
            if (node_degrees[e.from] > 2)
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
        BOOST_FOREACH(const osm::intr_pair &ip, intersections)
        {
            const intersection&  i = ip.second;
            const node          &intersection_node(nodes[i.id_from_node]);

            std::map<edge*, create_intersections::Edge_Offset*> edges_to_offsets;
            std::vector<create_intersections::Edge_Offset> edge_offsets;

            //
            // Create a vector of edge offsets and a map of edges to their offsets
            //
            BOOST_FOREACH(edge* an_edge, i.edges_starting_here)
            {
                create_intersections::Edge_Offset an_edge_offset;

                //Compute the vector
                vec2f vec(an_edge->shape[1]->xy[0] - an_edge->shape[0]->xy[0],
                          an_edge->shape[1]->xy[1] - an_edge->shape[0]->xy[1]);
                vec                            *= 1.0/tvmet::norm2(vec);
                assert(!isnan(vec[0]));
                assert(!isnan(vec[1]));


                an_edge_offset.approach_vector  = vec;

                //Record the number of lanes
                an_edge_offset.num_of_lanes = an_edge->type->nolanes;

                //Calculate the angle
                float x = an_edge_offset.approach_vector[0];
                float y = an_edge_offset.approach_vector[1];
                an_edge_offset.angle = atan2(y, x);

                //Initialize the offset to -inf (essentially)
                an_edge_offset.offset = -1*std::numeric_limits<float>::max();

                //Record the edge used to create this structure
                an_edge_offset.parent_edge = an_edge;

                //Save the edge offset
                edge_offsets.push_back(an_edge_offset);
            }

            BOOST_FOREACH(edge* an_edge, i.edges_ending_here)
            {
                create_intersections::Edge_Offset an_edge_offset;

                //Compute the vector
                size_t last = an_edge->shape.size() - 1;
                size_t penultimate = an_edge->shape.size() - 2;
                vec2f vec(an_edge->shape[penultimate]->xy[0] - an_edge->shape[last]->xy[0],
                          an_edge->shape[penultimate]->xy[1] - an_edge->shape[last]->xy[1]);
                assert(!isnan(vec[0]));
                assert(!isnan(vec[1]));
                vec                            *= 1.0/tvmet::norm2(vec);

                an_edge_offset.approach_vector  = vec;

                //Calculate the angle
                float x = an_edge_offset.approach_vector[0];
                float y = an_edge_offset.approach_vector[1];
                an_edge_offset.angle = atan2(y, x);

                //Initialize the offset to -inf
                an_edge_offset.offset = -1*std::numeric_limits<float>::max();

                //Record the number of lanes
                an_edge_offset.num_of_lanes = an_edge->type->nolanes;

                //Record the edge used to create this structure
                an_edge_offset.parent_edge = an_edge;

                edge_offsets.push_back(an_edge_offset);
            }

            sort(edge_offsets.begin(), edge_offsets.end(), create_intersections::Edge_Offset::clockwise);

            assert(edge_offsets.size() > 1);
            const float min_radius = 5;
            for (size_t i = 0; i < edge_offsets.size(); i++)
            {
                int clockwise_neighbor = (i == edge_offsets.size() - 1? 0 : i + 1);

                int no_lanes = std::max(edge_offsets[i].num_of_lanes, edge_offsets[clockwise_neighbor].num_of_lanes);

                float intersection_min = no_lanes * lane_width;

                vec2f clockwise_perp_a;
                clockwise_perp_a[0]  = -edge_offsets[i].approach_vector[1];
                clockwise_perp_a[1]  = edge_offsets[i].approach_vector[0];
                clockwise_perp_a    *= (no_lanes*lane_width + min_radius);

                vec2f cclockwise_perp_b;
                cclockwise_perp_b[0]  = edge_offsets[clockwise_neighbor].approach_vector[1];
                cclockwise_perp_b[1]  = -edge_offsets[clockwise_neighbor].approach_vector[0];
                cclockwise_perp_b    *= (no_lanes*lane_width + min_radius);

                vec2f sum(clockwise_perp_a - cclockwise_perp_b);
                float len = norm2(sum);
                float theta = edge_offsets[clockwise_neighbor].angle - edge_offsets[i].angle;

                if (theta < 0){
                    theta += 2*M_PI;
                }

                float offset = len / (2.0f * sin(theta / 2.0f));

                if (offset > edge_offsets[i].offset)
                {
                    //Store the max of the offset and a minimum intersection
                    edge_offsets[i].offset = std::max(offset, intersection_min);
                }
                if (offset > edge_offsets[clockwise_neighbor].offset)
                {
                    //Store the max of the offset and a minimum intersection
                    edge_offsets[clockwise_neighbor].offset = std::max(offset, intersection_min);
                }
            }


            BOOST_FOREACH(create_intersections::Edge_Offset& an_offset, edge_offsets)
            {
                edges_to_offsets[an_offset.parent_edge] = &an_offset;
            }

            //Pull back roads
            BOOST_FOREACH(edge* edge_p, i.edges_starting_here)
            {
                edge& e = (*edge_p);

                float offset = edges_to_offsets[edge_p]->offset;

                assert(!isnan(offset));

                double len_thus_far = 0;
                double _len         = 0;
                //Remove elements until the next segment's length is greater than offset - previous segments.
                int    new_start    = -1;
                do
                {
                    //TODO could go infinite for tiny roads.
                    len_thus_far                 += _len;
                    new_start++;
                    _len                          = planar_distance(e.shape[new_start + 1]->xy, e.shape[new_start]->xy);
                }
                while (len_thus_far + _len<= offset); //TODO degenerate case when equal.

                //Update node degree count.
                for (int i = 1; i <= new_start; i++)
                    node_degrees[e.shape[i]->id]--;

                //Modify geometry to make room for intersection.
                vec3f start_seg(e.shape[new_start + 1]->xy - e.shape[new_start]->xy);
                start_seg[2] = 0.0f;
                const double len = length(start_seg);
                double factor = (len - (offset - len_thus_far))/len;
                start_seg *= factor;

                e.shape[new_start]        = new node(*e.shape[new_start]);
                e.shape[new_start]->id    = e.shape[0]->id;
                e.shape[new_start]->xy[0] = e.shape[new_start + 1]->xy[0] - start_seg[0];
                e.shape[new_start]->xy[1] = e.shape[new_start + 1]->xy[1] - start_seg[1];
                e.shape[new_start]->xy[2] = intersection_node.xy[2];

                assert(!isnan(e.shape[new_start]->xy[0]));
                assert(!isnan(e.shape[new_start]->xy[1]));
                assert(!isnan(e.shape[new_start]->xy[2]));


                if (new_start != 0)
                    e.shape.erase(e.shape.begin(), e.shape.begin() + new_start);
            }

            BOOST_FOREACH(edge* edge_p, i.edges_ending_here)
            {
                edge& e = (*edge_p);

                float offset = edges_to_offsets[edge_p]->offset;

                double len_thus_far = 0;
                double _len         = 0;
                int    new_end      = e.shape.size();
                do
                {
                    //TODO could go infinite for tiny roads.
                    len_thus_far += _len;
                    new_end--;
                    _len          = planar_distance(e.shape[new_end]->xy, e.shape[new_end-1]->xy);
                }
                while(len_thus_far + _len <= offset);

                //Update node degree count
                //Don't change count for the last node, as we use its id.
                for (int i = new_end; i < static_cast<int>(e.shape.size()) - 1; i++)
                    node_degrees[e.shape[i]->id]--;

                vec3f end_seg = e.shape[new_end]->xy;

                end_seg -=  e.shape[new_end - 1]->xy;

                const double len = planar_distance(e.shape[new_end]->xy, e.shape[new_end - 1]->xy);
                double factor = (len - (offset - len_thus_far))/len;

                end_seg                 *= factor;
                e.shape[new_end]         = new node(*e.shape[new_end]);
                e.shape[new_end]->id     = e.shape[e.shape.size() - 1]->id;
                e.shape[new_end]->xy[0]  = e.shape[new_end - 1]->xy[0] + end_seg[0];
                e.shape[new_end]->xy[1]  = e.shape[new_end - 1]->xy[1] + end_seg[1];
                e.shape[new_end]->xy[2]  = intersection_node.xy[2];

                assert(!isnan(e.shape[new_end]->xy[0]));
                assert(!isnan(e.shape[new_end]->xy[1]));
                assert(!isnan(e.shape[new_end]->xy[2]));

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
        {
            b->shape[i]->edges_including.erase(find(b->shape[i]->edges_including.begin(), b->shape[i]->edges_including.end(), b));
            b->shape[i]->edges_including.push_back(a);
            a->shape.push_back(b->shape[i]);
        }

        a->to = b->to;

        assert(a->shape.size() == a_size + b_size - 1);
    }

    void edge::remove_duplicate_nodes()
    {
        std::vector<node*> new_node_list;
        str last_id = shape[0]->id;
        vec3f last_vec = shape[0]->xy;
        new_node_list.push_back(shape[0]);
        for(size_t i = 1; i < shape.size(); i++)
        {
            if ((shape[i]->id != last_id)
                and ((shape[i]->xy[0] != last_vec[0])
                     or
                     (shape[i]->xy[1] != last_vec[1])))
            {
                new_node_list.push_back(shape[i]);
                last_id = shape[i]->id;
                last_vec = shape[i]->xy;
            }
        }
        shape.clear();
        for(size_t i = 0; i < new_node_list.size(); i++)
            shape.push_back(new_node_list[i]);
    }

    void edge::reverse()
    {
        std::swap(from, to);
        std::reverse(shape.begin(), shape.end());
    }

    void network::join_logical_roads()
    {
        compute_node_degrees();
        edges_including_rebuild();
        node_degrees_and_edges_agree();
        std::vector<str> edges_to_delete;

        BOOST_FOREACH(osm::node_pair &np, nodes)
        {
            if (node_degrees[np.first] == 2)
            {
                assert(np.second.edges_including.size() < 3);
                if(np.second.edges_including.size() == 2)
                {
                    edge* e = np.second.edges_including[0];
                    edge* o = np.second.edges_including[1];

                    if (o == e)
                        continue;

                    if ((nodes[o->to].id == nodes[e->from].id) and (o->to == np.first))
                    {
                        std::swap(o, e);
                    }

                    if ((nodes[e->to].id == nodes[o->from].id) and (e->to == np.first))
                    {
                        node_degrees[e->to]--;

                        int e_size = e->shape.size();
                        int o_size = o->shape.size();
                        join(e, o);
                        assert(static_cast<int>(e->shape.size()) == e_size + o_size - 1);

                        edges_to_delete.push_back(o->id);
                        // std::vector<edge*>::iterator ei_it = find(np.second.edges_including.begin(), np.second.edges_including.end(), o);

                        // assert(np.second.edges_including.end() != ei_it);
                        // np.second.edges_including.erase(ei_it);
                    }
                    else if ((nodes[e->to].id == nodes[o->to].id) and (e->to == np.first))
                    {
                        node_degrees[e->to]--;

                        int e_size = e->shape.size();
                        int o_size = o->shape.size();

                        o->reverse();
                        join(e, o);

                        assert(static_cast<int>(e->shape.size()) == e_size + o_size - 1);
                        // std::vector<edge>::iterator j_it = find(edges.begin(), edges.end(), *o);
                        // assert(j_it != edges.end());
                        // edges.erase(j_it);
                        edges_to_delete.push_back(o->id);
                        // std::vector<edge*>::iterator ei_it = find(np.second.edges_including.begin(), np.second.edges_including.end(), o);
                        // assert(np.second.edges_including.end() != ei_it);
                        // np.second.edges_including.erase(ei_it);
                    }
                    else if ((nodes[e->from].id == nodes[o->from].id) and (e->from == np.first))
                    {
                        node_degrees[e->from]--;

                        int e_size = e->shape.size();
                        int o_size = o->shape.size();

                        e->reverse();
                        join(e, o);

                        assert(static_cast<int>(e->shape.size()) == e_size + o_size - 1);
                        // std::vector<edge>::iterator j_it = find(edges.begin(), edges.end(), *o);
                        // assert(j_it != edges.end());
                        // edges.erase(j_it);
                        edges_to_delete.push_back(o->id);

                        // std::vector<edge*>::iterator ei_it = find(np.second.edges_including.begin(), np.second.edges_including.end(), o);
                        // assert(np.second.edges_including.end() != ei_it);
                        // np.second.edges_including.erase(ei_it);
                    }
                }
            }
        }

        for(size_t i = 0; i < edges_to_delete.size(); i++)
        {
            for(size_t j = 0; j < edges.size(); j++)
            {
                if (edges[j].id == edges_to_delete[i])
                {
                    edges.erase(edges.begin() + j);
                    break;
                }
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
        //        std::map<str, std::vector<str> > road_split_points;
        std::map<str, std::vector<int> > road_split_points;
        BOOST_FOREACH(edge &ep, edges)
        {
            //Check nodes for split points, but skip the first and last
            for (int i = 1; i < static_cast<int>(ep.shape.size()) - 1; i++)
            {
                if (node_degrees[ep.shape[i]->id] > 1)
                {
                    road_split_points[ep.id].push_back(i);
                }
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
            BOOST_FOREACH(int split_index, road_split_points[_edge.id])
            {
                if (not _first)
                {
                    new_edges.push_back(copy_no_shape(_edge));
                    new_edges.back().from = _edge.shape[node_index]->id;
                    new_edges.back().shape.push_back(&nodes[_edge.shape[node_index]->id]);
                }

                //Increase the node degree as the road is being split.
                node_degrees[_edge.shape[split_index]->id]++;

                //Add each node to new edge up to, and including, the next splitter
                //Set the first node of the new edge

                while (node_index != split_index)
                {
                    node_index++;
                    if (not _first)
                    {
                        //Add nodes to new road
                        new_edges.back().shape.push_back(&nodes[_edge.shape[node_index]->id]);
                        new_edges.back().to = _edge.shape[node_index]->id;
                    }
                }

                if (not _first)
                    assert(new_edges.back().shape.size() > 1);

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

                for (;node_index < static_cast<int>(_edge.shape.size()); node_index++)
                {
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
        // this is a goofy way to do this, we should use a hash or something
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
                e_type->nolanes = 3;
                e_type->oneway  = 1;
            }
            else if(e.highway_class == "motorway_link")
            {
                e_type->speed   = 30;
                e_type->nolanes = 1;
                e_type->oneway  = 1;
            }
            else if(e.highway_class == "residential")
            {
                e_type->speed   = 30;
                e_type->nolanes = 1;
                e_type->oneway  = 0;
            }
            else if(e.highway_class == "primary")
            {
                e_type->nolanes = 1;
                e_type->oneway  = 0;
                e_type->speed   = 50;
            }
            else if(e.highway_class == "secondary")
            {
                e_type->speed   = 40;
                e_type->nolanes = 1;
                e_type->oneway  = 0;
            }
            else if(e.highway_class == "service")
            {
                e_type->speed   = 25;
                e_type->nolanes = 1;
                e_type->oneway  = 0;
            }
            else if(e.highway_class == "primary_link")
            {
                e_type->speed   = 30;
                e_type->nolanes = 1;
                e_type->oneway  = 1;
            }
            else if(e.highway_class == "secondary_link")
            {
                e_type->speed   = 30;
                e_type->nolanes = 1;
                e_type->oneway  = 1;
            }
            else if(e.highway_class == "urban")
            {
                //Classes added for grid
                e_type->speed   = 30;
                e_type->nolanes = 2;
                e_type->oneway  = 0;
            }
            e_type->speed *= MIPH_TO_MEPS;
        }

        BOOST_FOREACH(osm::edge &e, edges)
        {
            assert(e.type->nolanes > 0);
            assert(e.type->nolanes < 10);
        }
    }
}
