#ifndef _HWM_NETWORK_HPP_
#define _HWM_NETWORK_HPP_

#include "partition01.hpp"
#include "polyline_road.hpp"
#include "arc_road.hpp"
#include "libroad_common.hpp"
#include "sumo_network.hpp"

namespace hwm
{
    struct network;

    struct road
    {
        bool xml_read (network &n, xmlpp::TextReader &reader, const vec3f &scale=vec3f(1.0f, 1.0f, 1.0f));
        void xml_write(xmlpp::Element *elt) const;
        bool check() const;

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
            virtual terminus* clone() const;
            virtual bool xml_read (network &n, const lane *parent, xmlpp::TextReader &reader, const str &name);
            virtual void xml_write(xmlpp::Element *elt, const str &name) const;
            virtual bool check(bool start, const lane *parent) const;
            virtual lane *incident(bool start) const;
        };

        struct intersection_terminus : public terminus
        {
            intersection_terminus() : adjacent_intersection(0), intersect_in_ref(-1)
            {}

            intersection_terminus(intersection *i, int ref) : adjacent_intersection(i), intersect_in_ref(ref)
            {}

            virtual intersection_terminus* clone() const;
            virtual bool xml_read (network &n, const lane *parent, xmlpp::TextReader &reader, const str &name);
            virtual void xml_write(xmlpp::Element *elt, const str &name) const;
            virtual bool check(bool start, const lane *parent) const;
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

            virtual lane_terminus* clone() const;
            virtual bool xml_read (network &n, const lane *parent, xmlpp::TextReader &reader, const str &name);
            virtual void xml_write(xmlpp::Element *elt, const str &name) const;
            virtual bool check(bool start, const lane *parent) const;
            virtual lane *incident(bool start) const;

            lane *adjacent_lane;
        };

        struct road_membership
        {
            bool xml_read (network &n, xmlpp::TextReader &reader);
            void xml_write(xmlpp::Element *elt) const;
            bool check() const;
            bool empty() const;

            void scale_offsets(float lane_width);

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
            bool xml_read (network &n, xmlpp::TextReader &reader);
            void xml_write(xmlpp::Element *elt) const;
            bool check() const;
            bool empty() const;

            typedef partition01<adjacency>  intervals;
            lane                           *neighbor;
            intervals::interval_t           neighbor_interval;
        };

        bool xml_read (network &n, xmlpp::TextReader &reader);
        void xml_write(xmlpp::Element *elt) const;
        bool check() const;
        void scale_offsets(float lane_width);

        void make_mesh(std::vector<vertex> &verts, std::vector<vec3u> &faces, float lane_width, float resolution) const;

        float   length     () const;
        vec3f   point      (float t, float offset=0.0f, const vec3f &up=vec3f(0, 0, 1)) const;
        mat3x3f frame      (float t,                    const vec3f &up=vec3f(0, 0, 1)) const;
        mat4x4f point_frame(float t, float offset=0.0f, const vec3f &up=vec3f(0, 0, 1)) const;

        lane *upstream_lane()   const;
        lane *downstream_lane() const;

        str                         id;
        road_membership::intervals  road_memberships;
        adjacency::intervals        left;
        adjacency::intervals        right;
        terminus                   *start;
        terminus                   *end;
        float                       speedlimit;
    };

    struct intersection
    {
        intersection() : current_state(0)
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

                bool check(const intersection &parent) const;
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

            bool xml_read (xmlpp::TextReader &reader);
            void xml_write(const size_t id, xmlpp::Element *elt) const;
            bool check(const intersection &parent) const;

            state_pair_in        &in_pair();
            const state_pair_in  &in_pair() const;
            state_pair_out       &out_pair();
            const state_pair_out &out_pair() const;

            float               duration;
            state_pair_set      state_pairs;
        };

        bool xml_read (network &n, xmlpp::TextReader &reader);
        void xml_write(xmlpp::Element *elt) const;
        bool check() const;

        void translate(const vec3f &o);
        void build_shape(float lane_width);

        lane *downstream_lane(int incoming_ref) const;
        lane *  upstream_lane(int outgoing_ref) const;

        str                id;
        std::vector<lane*> incoming;
        std::vector<lane*> outgoing;
        std::vector<state> states;
        int                current_state;
        std::vector<vec3f> shape;
        vec3f              center;
    };

    struct network
    {
        network() {};
        network(const network &n);

        void copy(const network &n);

        network &operator=(const network &n);

        bool xml_read (xmlpp::TextReader &reader);
        void xml_write(const char *filename) const;
        void xml_write(xmlpp::Element *elt)  const;
        bool check() const;
        void scale_offsets(float lane_width);
        void build_intersections(float lane_width);

        void center(bool z=false);

        void translate(const vec3f &o);

        void bounding_box(vec3f &low, vec3f &high) const;

        void build_intersection_roads();

        vec3f                       scale;
        str                         name;
        float                       gamma;
        strhash<road>::type         roads;
        strhash<lane>::type         lanes;
        strhash<intersection>::type intersections;
        strhash<road>::type         i_roads;
        strhash<lane>::type         i_lanes;
    };

    network load_xml_network(const char *filename, const vec3f &scale=vec3f(1.0f, 1.0f, 1.0f));
    void    write_xml_network(const network &n, const char *filename);

    network from_sumo(const str &name, float gamma, const sumo::network &n);
};
#endif
