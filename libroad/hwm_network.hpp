#ifndef _HWM_NETWORK_HPP_
#define _HWM_NETWORK_HPP_

#include "partition01.hpp"
#include "polyline_road.hpp"
#include "arc_road.hpp"
#include "libroad_common.hpp"
#include "sumo_network.hpp"

namespace hwm
{
    struct road
    {
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
        struct terminus
        {
            bool check(bool start, const lane *parent) const;

            intersection *inters;
            int           intersect_in_ref;
        };

        struct road_membership
        {
            bool check() const;
            bool empty() const;

            void scale_offsets(float lane_width);

            float   length     () const;
            vec3f   point      (float t, const vec3f &up=vec3f(0, 0, 1)) const;
            mat3x3f frame      (float t, const vec3f &up=vec3f(0, 0, 1)) const;
            mat4x4f point_frame(float t, const vec3f &up=vec3f(0, 0, 1)) const;

            typedef partition01<road_membership>  intervals;
            road                                 *parent_road;
            intervals::interval_t                 interval;
            float                                 lane_position;
        };

        struct adjacency
        {
            bool check() const;
            bool empty() const;

            typedef partition01<adjacency>  intervals;
            lane                           *neighbor;
            intervals::interval_t           neighbor_interval;
        };

        bool check() const;
        void scale_offsets(float lane_width);

        void make_mesh(std::vector<vertex> &verts, std::vector<vec3u> &faces, float lane_width, float resolution) const;

        float   length     () const;
        vec3f   point      (float t, const vec3f &up=vec3f(0, 0, 1)) const;
        mat3x3f frame      (float t, const vec3f &up=vec3f(0, 0, 1)) const;
        mat4x4f point_frame(float t, const vec3f &up=vec3f(0, 0, 1)) const;

        str                        id;
        road_membership::intervals road_memberships;
        adjacency::intervals       left;
        adjacency::intervals       right;
        terminus                   start;
        terminus                   end;
        float                      speedlimit;
    };

    struct intersection
    {
        struct state
        {
            struct in_id
            {
                bool check() const;

                int   in_ref;
                lane *fict_lane;
            };

            struct out_id
            {
                bool check() const;

                int   out_ref;
                lane *fict_lane;
            };

            bool check() const;

            enum {STARVATION=-1, STOP=-1};

            float duration;
            std::vector<out_id> in_states;
            std::vector<in_id>  out_states;
        };

        bool check() const;

        str                id;
        std::vector<lane*> incoming;
        std::vector<lane*> outgoing;
        std::vector<state> states;
        int                current_state;
    };

    struct network
    {
        network() {};
        network(const network &n);

        void copy(const network &n);

        network &operator=(const network &n);

        bool check() const;
        void scale_offsets(float lane_width);

        void center();

        void translate(const vec3f &o);

        void bounding_box(vec3f &low, vec3f &high) const;

        str                         name;
        float                       gamma;
        strhash<road>::type         roads;
        strhash<lane>::type         lanes;
        strhash<intersection>::type intersections;
    };

    network load_xml_network(const char *filename);
    void    write_xml_network(const network &n, const char *filename);

    network from_sumo(const str &name, float gamma, const sumo::network &n);
};
#endif
