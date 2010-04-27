#ifndef _HWM_NETWORK_HPP_
#define _HWM_NETWORK_HPP_

#include "partition01.hpp"
#include "polyline_road.hpp"
#include "arc_road.hpp"
#include "libroad_common.hpp"
#include "svg_helper.hpp"
#include "sumo_network.hpp"
#include "osm_network.hpp"
#include "hwm_texture_gen.hpp"

namespace hwm
{
    static const float tire_static_friction = 0.7f;

    float maximum_cornering_speed(float radius, float g, float static_friction);

    struct network;

    struct road
    {
        void xml_read (const vec3f &scale, xmlpp::TextReader &reader);
        void xml_write(xmlpp::Element *elt) const;
        void check() const;

        void translate(const vec3f &o);
        void bounding_box(vec3f &low, vec3f &high) const;

        str      id;
        str      name;
        arc_road rep;
    };

    struct intersection;

    struct lane
    {
        lane();
        lane(const lane &l);

        ~lane()
        {
            delete start;
            delete end;
        }

        struct terminus
        {
            virtual void update_pointers(network &n);
            virtual terminus* clone() const;
            virtual void xml_read (network &n, const lane *parent, xmlpp::TextReader &reader, const str &name);
            virtual void xml_write(xmlpp::Element *elt, const str &name) const;
            virtual void check(bool start, const lane *parent) const;
            virtual lane *incident(bool start) const;
        };

        struct intersection_terminus : public terminus
        {
            intersection_terminus() : adjacent_intersection(0), intersect_in_ref(-1)
            {}

            intersection_terminus(intersection *i, int ref) : adjacent_intersection(i), intersect_in_ref(ref)
            {}

            virtual void update_pointers(network &n);
            virtual intersection_terminus* clone() const;
            virtual void xml_read (network &n, const lane *parent, xmlpp::TextReader &reader, const str &name);
            virtual void xml_write(xmlpp::Element *elt, const str &name) const;
            virtual void check(bool start, const lane *parent) const;
            virtual lane *incident(bool start) const;

            intersection *adjacent_intersection;
            int           intersect_in_ref;
        };

        struct lane_terminus : public terminus
        {
            lane_terminus() : adjacent_lane(0)
            {}

            lane_terminus(lane* l) : adjacent_lane(l)
            {}

            virtual void update_pointers(network &n);
            virtual lane_terminus* clone() const;
            virtual void xml_read (network &n, const lane *parent, xmlpp::TextReader &reader, const str &name);
            virtual void xml_write(xmlpp::Element *elt, const str &name) const;
            virtual void check(bool start, const lane *parent) const;
            virtual lane *incident(bool start) const;

            lane *adjacent_lane;
        };

        struct road_membership
        {
            void xml_read (network &n, xmlpp::TextReader &reader);
            void xml_write(xmlpp::Element *elt) const;
            void check() const;
            bool empty() const;

            float   length      () const;
            vec3f   point       (float t, float offset=0.0f, const vec3f &up=vec3f(0, 0, 1)) const;
            mat3x3f frame       (float t,                    const vec3f &up=vec3f(0, 0, 1)) const;
            mat4x4f point_frame (float t, float offset=0.0f, const vec3f &up=vec3f(0, 0, 1)) const;

            typedef partition01<road_membership>  intervals;
            road                                 *parent_road;
            intervals::interval_t                 interval;
            float                                 lane_position;
        };

        struct adjacency
        {
            void xml_read (network &n, xmlpp::TextReader &reader);
            void xml_write(xmlpp::Element *elt) const;
            void check() const;
            bool empty() const;

            typedef partition01<adjacency>  intervals;
            lane                           *neighbor;
            intervals::interval_t           neighbor_interval;
        };

        void xml_read (network &n, xmlpp::TextReader &reader);
        void xml_write(xmlpp::Element *elt) const;
        void check() const;
        void auto_scale_memberships();

        void make_mesh(std::vector<vertex> &verts, std::vector<vec3u> &faces, float lane_width, float resolution) const;
        path svg_arc_path (float lane_width) const;
        path svg_poly_path(float lane_width) const;

        float   length     () const;
        vec3f   point      (float t, float offset=0.0f, const vec3f &up=vec3f(0, 0, 1)) const;
        mat3x3f frame      (float t,                    const vec3f &up=vec3f(0, 0, 1)) const;
        mat4x4f point_frame(float t, float offset=0.0f, const vec3f &up=vec3f(0, 0, 1)) const;

        lane *left_adjacency(float &param)  const;
        lane *right_adjacency(float &param) const;

        lane *upstream_lane()   const;
        lane *downstream_lane() const;

        template<typename T>
        T *user_data()
        {
            return reinterpret_cast<T*>(user_datum);
        }

        template<typename T>
        const T *user_data() const
        {
            return reinterpret_cast<const T*>(user_datum);
        }

        str                         id;
        road_membership::intervals  road_memberships;
        adjacency::intervals        left;
        adjacency::intervals        right;
        terminus                   *start;
        terminus                   *end;
        float                       speedlimit;
        bool                        active;
        void                       *user_datum;
    };

    struct intersection
    {
        intersection() : locked(false), current_state(0), state_time(0)
        {}

        struct state
        {
            struct in
            {};
            struct out
            {};

            struct state_pair
            {
                state_pair()
                    : fict_lane(0)
                {}
                state_pair(const int i, const int o)
                    : in_idx(i), out_idx(o), fict_lane(0)
                {}
                state_pair(const int i, const int o, lane *l)
                    : in_idx(i), out_idx(o), fict_lane(l)
                {}

                void check(const intersection &parent) const;
                int   in_idx;
                int   out_idx;
                lane *fict_lane;
            };

            typedef boost::multi_index_container<
                state_pair,
                boost::multi_index::indexed_by<
                    boost::multi_index::hashed_unique<boost::multi_index::tag<in>, boost::multi_index::member<state_pair,int,&state_pair::in_idx> >,
                    boost::multi_index::hashed_unique<boost::multi_index::tag<out>,boost::multi_index::member<state_pair,int,&state_pair::out_idx> >
                    >
            > state_pair_set;

            typedef intersection::state::state_pair_set::index<intersection::state::in >::type state_pair_in;
            typedef intersection::state::state_pair_set::index<intersection::state::out>::type state_pair_out;

            void xml_read (xmlpp::TextReader &reader);
            void xml_write(const size_t id, xmlpp::Element *elt) const;
            void check(const intersection &parent) const;

            void translate(const vec3f &o);
            void build_fictitious_lanes(const intersection &parent);

            state_pair_in        &in_pair();
            const state_pair_in  &in_pair() const;
            state_pair_out       &out_pair();
            const state_pair_out &out_pair() const;

            void activate();
            void deactivate();

            float               duration;
            state_pair_set      state_pairs;
            strhash<road>::type fict_roads;
            strhash<lane>::type fict_lanes;
        };

        void xml_read (network &n, xmlpp::TextReader &reader);
        void xml_write(xmlpp::Element *elt) const;
        void check() const;

        void translate(const vec3f &o);
        void build_shape(float lane_width);
        void build_fictitious_lanes();

        lane *downstream_lane(int incoming_ref) const;
        lane *  upstream_lane(int outgoing_ref) const;

        void advance_state();
        void lock();
        void unlock();

        str                id;
        std::vector<lane*> incoming;
        std::vector<lane*> outgoing;
        std::vector<state> states;
        bool               locked;
        size_t             current_state;
        float              state_time;
        std::vector<vec3f> shape;
        vec3f              center;
    };

    typedef strhash<road>::type         road_map;
    typedef strhash<lane>::type         lane_map;
    typedef strhash<intersection>::type intersection_map;

    typedef road_map::value_type         road_pair;
    typedef lane_map::value_type         lane_pair;
    typedef intersection_map::value_type intersection_pair;

    struct network
    {
        static const int SVG_ROADS=1, SVG_LANES=4, SVG_ARCS=8, SVG_CIRCLES=16;

        network() {};
        network(const network &n);

        void copy(const network &n);

        network &operator=(const network &n);

        void xml_read (xmlpp::TextReader &reader, const vec3f &scale=vec3f(1.0f,1.0f,1.0f));
        void xml_write(const char *filename) const;
        void xml_write(xmlpp::Element *elt)  const;
        void svg_write(const char *filename, const int flags) const;

        void check() const;
        void build_intersections();
        void build_fictitious_lanes();
        void auto_scale_memberships();

        void center(bool z=false);

        void translate(const vec3f &o);

        void bounding_box(vec3f &low, vec3f &high) const;

        str              name;
        float            gamma;
        float            lane_width;
        road_map         roads;
        lane_map         lanes;
        intersection_map intersections;
    };

    struct network_aux
    {
        struct road_rev_map
        {
            struct lane_entry
            {
                lane_entry();
                lane_entry(const lane* l, const lane::road_membership *rm);

                const hwm::lane                  *lane;
                const hwm::lane::road_membership *membership;
            };

            struct lane_cont : public std::map<const float, const lane_entry>
            {
#if HAVE_CAIRO
                const std::string write_texture(tex_db &tdb) const;
#endif

                void make_mesh(std::vector<vertex> &vrts, std::vector<vec3u> &fcs, const vec2f &interval, const float lane_width) const;
            };

            road_rev_map();
            road_rev_map(const road *r);

            void add_lane(const lane *r, const lane::road_membership *rm);
            void print() const;

            partition01<lane_cont>  lane_map;
            const hwm::road        *road;
        };

        struct point_tan
        {
            vec3f point;
            vec3f tan;
        };

        typedef std::map<float, point_tan>  incident_point_tans;

        struct road_store
        {
            road_store(const hwm::road *r, const bool as);

            const hwm::road     *road;
            bool                 at_start;
            incident_point_tans  ipt;
        };

        struct road_is_cnt : public std::vector<road_store>
        {
            road_store &find(const hwm::road *r, bool as);
        };

        network_aux(const network &n);

        void road_objs(std::ostream &os) const;

        void intersection_obj(std::ostream &os, const hwm::intersection &is) const;
        void network_obj(const std::string &path) const;

        strhash<road_rev_map>::type  rrm;
        const network               &net;
   };

    network load_xml_network(const char *filename, const vec3f &scale=vec3f(1.0f, 1.0f, 1.0f));
    void    write_xml_network(const network &n, const char *filename);

    network from_sumo(const str &name, float gamma, float lane_width, const sumo::network &n);
    network from_osm (const str &name, float gamma, float lane_width,       osm::network &n);
};
#endif
