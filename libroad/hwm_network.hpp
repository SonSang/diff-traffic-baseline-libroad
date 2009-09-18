#ifndef _SUMO_NETWORK_HPP_
#define _SUMO_NETWORK_HPP_

#include <vector>
#include <map>
#include <tvmet/Vector.h>
#include <glibmm/ustring.h>
#include <libxml++/libxml++.h>
#include <libxml++/parsers/textreader.h>

#include "partition01.hpp"
#include "xml_util.hpp"
#include "polyline_road.hpp"

namespace hwm
{
    struct road
    {
        str           id;
        str           name;
        polyline_road rep;
    };

    struct intersection;

    struct lane
    {
        struct terminus
        {
            intersection *inters;
            int           intersect_in_ref;
        };

        struct road_membership
        {
            road *parent_road;
            float interval[2];
            float lane_position;

            typedef partition01<road_membership> intervals;
        };

        struct adjacency
        {
            lane *neighbor;
            float neighbor_interval[2];
            typedef partition01<adjacency> intervals;
        };

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
                int   in_ref;
                lane *fict_lane;
            };

            struct out_id
            {
                int   out_ref;
                lane *fict_lane;
            };

            enum {STARVATION=-1, STOP=-1};

            std::vector<out_id> in_states;
            std::vector<in_id>  out_states;
        };

        str                id;
        std::vector<lane*> incoming;
        std::vector<lane*> outgoing;
        std::vector<state> states;
        int                current_state;
    };

    struct network
    {
        str                         name;
        float                       gamma;
        strhash<road>::type         roads;
        strhash<lane>::type         lanes;
        strhash<intersection>::type intersections;

        road         *retreive_road(const str &id);
        lane         *retreive_lane(const str &id);
        intersection *retreive_intersection(const str &id);

        template <class T>
        bool xml_read(partition01<T> &part, xmlpp::TextReader &reader, const str &tag);
        bool xml_read(lane::road_membership &rm, xmlpp::TextReader &reader);
        bool xml_read(lane::adjacency &la, xmlpp::TextReader &reader);
        bool xml_read(lane::terminus &lt, xmlpp::TextReader &reader, const str &endtag);
        bool xml_read(road &r, xmlpp::TextReader &reader);
        bool xml_read(lane &l, xmlpp::TextReader &reader);
        bool xml_read(intersection &i, xmlpp::TextReader &reader);

        bool xml_read(const char *filename);
    };
};
#endif
