#ifndef _OSM_NETWORK_HPP_
#define _OSM_NETWORK_HPP_

#include "libroad_common.hpp"
#include <vector>

namespace osm
{
    struct edge;

    struct node
    {
        node(){ is_overpass = false; ramp_merging_point = NULL;}

        str                id;
        vec3f              xy;
        std::vector<edge*> edges_including; //Currently not maintained
        bool               is_overpass;
        node*              ramp_merging_point;
    };

    struct edge_type
    {
        str    id;
        int    nolanes;
        double speed;
        int    priority;
        bool   oneway;
    };

    struct shape_t : public std::vector<node*>
    {
    };

    struct edge
    {
        ~edge(){}

        bool operator==(const edge& e) {return id == e.id;}

        typedef enum {center, right} SPREAD;

        str        id;
        str        from;
        str        to;
        edge_type* type;
        shape_t    shape;
        SPREAD     spread;
        str        highway_class;

        struct lane
        {
            lane(float s, float e, float off, str id, bool offr):start_t(s), end_t(e), offset(off), ramp_id(id), offramp(offr){}

            float start_t;
            float end_t;
            float offset;
            str   ramp_id;
            bool  offramp;
        };

        std::vector<node*> overpass_nodes;
        std::vector<lane>  additional_lanes;

        void remove_duplicate_nodes();
        void reverse();
        float length() const;
    };

    struct intersection
    {
        std::vector<edge*> edges_ending_here;
        std::vector<edge*> edges_starting_here;
        str                id_from_node;
    };

    struct network
    {
        network()
        {}

        strhash<node>::type              nodes;
        strhash<edge_type>::type         types;
        strhash<edge>::type              edge_hash;
        strhash<intersection>::type      intersections;
        strhash<edge>::type              road_segs;
        strhash<int>::type               node_degrees;
        strhash<std::vector<str> >::type node_connections;
        std::vector<edge>                edges;

        vec2d center;
        vec2d topleft;
        vec2d bottomright;

        //Error checking functions
        void node_degrees_and_edges_agree();
        void intersection_check();
        void edges_check();
        void display_used_node_heights();
        void list_edges();

        void remove_duplicate_nodes();
        void edges_including_rebuild();
        bool out_of_bounds(const vec3f &) const;
        void clip_roads_to_bounds();
        void create_ramps();
        void populate_edges_from_hash();
        void populate_edge_hash_from_edges();
        void remove_small_roads(double min_len);
        void remove_highway_intersections();
        void create_grid(int, int, double, double);
        void scale_and_translate();
        void compute_node_heights();
        void check_edge(const edge &e) const;
        void check_node(const node &n) const;
        void check() const;
        void compute_edge_types();
        void draw_network() ;
        void create_intersections();
        void compute_node_degrees();
        void join_logical_roads();
        void split_into_road_segments();
        void join(edge* , edge*);
        void check_nodes();
        edge copy_no_shape(const edge& e);

        static size_t new_edges_id;
    };

    network load_xml_network(const char *osm_file);
}

inline std::ostream &operator<<(std::ostream &o, const osm::edge::SPREAD &t)
{
    switch(t)
    {
    case osm::edge::center:
        o << "center";
        break;
    case osm::edge::right:
        o << "right";
        break;
    default:
        o << "unknown_spread_type";
        break;
    }
    return o;
}

inline std::istream &operator>>(std::istream &i, osm::edge::SPREAD &t)
{
    str src;
    i >> src;

    if(src == "center")
        t = osm::edge::center;
    else
        t = osm::edge::right;

    return i;
}

template <class T>
static inline T* retrieve(typename strhash<T>::type &m, const str &id)
{
    typedef typename strhash<T>::type val;
    typename strhash<T>::type::iterator entry(m.find(id));
    if(entry == m.end())
        m.insert(entry, std::make_pair(id, T()));

    return &(m[id]);
}
#endif
