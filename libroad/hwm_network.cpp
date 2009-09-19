#include "hwm_network.hpp"
#include <iostream>
#include <cassert>

namespace hwm
{
    template <class T>
    bool network::xml_read(partition01<T> &part, xmlpp::TextReader &reader, const str &tag)
    {
        if(!(read_to_open(reader, "interval") &&
             read_to_open(reader, "base") &&
             read_to_open(reader, tag)))
            return false;

        typename partition01<T>::iterator new_elt(part.insert(0.0, T()));

        if(!xml_read(new_elt->second, reader))
            return false;

        bool res = read_to_close(reader, "base");
        while(res && !is_closing_element(reader, "interval"))
        {
            if(!read_skip_comment(reader))
                return false;

            if(is_opening_element(reader, "divider"))
            {
                float div;
                if(!get_attribute(div, reader, "value"))
                    return false;

                if(!read_to_open(reader, tag))
                    return false;

                typename partition01<T>::iterator new_elt(part.insert(div, T()));

                if(!xml_read(new_elt->second, reader))
                    return false;

                if(!read_to_close(reader, "divider"))
                    return false;
            }
        }

        return res;
    }

    bool network::xml_read(lane::road_membership &rm, xmlpp::TextReader &reader)
    {
        assert(is_opening_element(reader, "road_membership"));

        str ref;
        if(!(get_attribute(ref, reader, "parent_road_ref") &&
             get_attribute(rm.interval[0], reader, "interval_start") &&
             get_attribute(rm.interval[1], reader, "interval_end") &&
             get_attribute(rm.lane_position, reader, "lane_position")))
            return false;

        rm.parent_road = retreive_road(ref);

        return read_to_close(reader, "road_membership");
    }

    bool network::xml_read(lane::adjacency &la, xmlpp::TextReader &reader)
    {
        assert(is_opening_element(reader, "lane_adjacency"));

        str ref;
        int acount = get_attribute(ref, reader, "lane_ref") +
                     get_attribute(la.neighbor_interval[0], reader, "interval_start") +
                     get_attribute(la.neighbor_interval[1], reader, "interval_end");
        if(acount == 3)
            la.neighbor = retreive_lane(ref);
        else if(acount == 0)
            la.neighbor = 0;
        else
            return false;

        return read_to_close(reader, "lane_adjacency");
    }

    bool network::xml_read(lane::terminus &lt, xmlpp::TextReader &reader, const str &tag)
    {
        assert(is_opening_element(reader, tag));

        bool res = true;
        while(res && !(is_opening_element(reader, "dead_end") ||
                       is_opening_element(reader, "intersection_ref")))
            res = read_skip_comment(reader);

        if(!res)
            return false;

        if(is_opening_element(reader, "dead_end"))
        {
            lt.inters           = 0;
            lt.intersect_in_ref = -1;

            if(!read_to_close(reader, "dead_end"))
                return false;
        }
        else if(is_opening_element(reader, "intersection_ref"))
        {
            str ref;
            if(!(get_attribute(ref, reader, "ref")))
                return false;

            lt.inters = retreive_intersection(ref);
            lt.intersect_in_ref = -1;

            if(!read_to_close(reader, "intersection_ref"))
                return false;
        }

        return read_to_close(reader, tag);
    }

    bool network::xml_read(road &r, xmlpp::TextReader &reader)
    {
        assert(is_opening_element(reader, "road"));

        if(!(get_attribute(r.id,   reader, "id") &&
             get_attribute(r.name, reader, "name")))
            return false;

        if(!read_to_open(reader, "line_rep"))
            return false;

        if(!r.rep.xml_read(reader))
            return false;

        return read_to_close(reader, "road");
    }

    bool network::xml_read(lane &l, xmlpp::TextReader &reader)
    {
        assert(is_opening_element(reader, "lane"));

        if(!(get_attribute(l.id,         reader, "id") &&
             get_attribute(l.speedlimit, reader, "speedlimit")))
            return false;

        bool have_start     = false;
        bool have_end       = false;
        bool have_road_int  = false;
        bool have_adjacency = false;
        bool res            = true;
        while(res && !is_closing_element(reader, "lane"))
        {
            res = read_skip_comment(reader);

            if(is_opening_element(reader, "start"))
                res = have_start = xml_read(l.start, reader, "start");
            else if(is_opening_element(reader, "end"))
                res = have_end = xml_read(l.end, reader, "end");
            else if(is_opening_element(reader, "road_intervals"))
            {
                if(!xml_read(l.road_memberships, reader, "road_membership"))
                    return false;
                res = have_road_int = read_to_close(reader, "road_intervals");
            }
            else if(is_opening_element(reader, "adjacency_intervals"))
            {
                bool have_left  = false;
                bool have_right = false;
                while(res && !is_closing_element(reader, "adjacency_intervals"))
                {
                    res = read_skip_comment(reader);

                    if(is_opening_element(reader, "left"))
                    {
                        have_left = (xml_read(l.left, reader, "lane_adjacency") &&
                                     read_to_close(reader, "left"));
                    }
                    else if(is_opening_element(reader, "right"))
                    {
                        have_right = (xml_read(l.right, reader, "lane_adjacency") &&
                                      read_to_close(reader, "right"));
                    }
                }
                have_adjacency = res && have_left && have_right;
            }
        }

        return (res && have_start && have_end && have_road_int &&
                have_adjacency);
    }

    bool network::xml_read(intersection &i, xmlpp::TextReader &reader)
    {
        assert(is_opening_element(reader, "intersection"));

        if(!get_attribute(i.id, reader, "id"))
             return false;

        return read_to_close(reader, "intersection");
    }

    bool network::xml_read(const char *filename)
    {
        try
        {
            xmlpp::TextReader reader(filename);

            if(!read_skip_comment(reader))
                return false;

            if(!is_opening_element(reader, "network"))
                return false;

            str version;
            if(!get_attribute(version, reader, "version") ||
               version != "1.2")
                return false;

            if(!get_attribute(name, reader, "name"))
                return false;

            if(!get_attribute(gamma, reader, "gamma") ||
               gamma <= 0.0f ||
               gamma >= 1.0f)
                return false;

            bool have_roads         = false;
            bool have_lanes         = false;
            bool have_intersections = false;
            bool res                = true;
            while(res && !is_closing_element(reader, "network"))
            {
                res = read_skip_comment(reader);

                if(is_opening_element(reader, "roads"))
                    res = have_roads = read_map(*this, roads, reader, "road", "roads");
                else if(is_opening_element(reader, "lanes"))
                    res = have_lanes = read_map(*this, lanes, reader, "lane", "lanes");
                else if(is_opening_element(reader, "intersections"))
                    res = have_intersections = read_map(*this, intersections, reader, "intersection", "intersections");
            }

            if(!res)
                std::cerr << "Error at line: " << reader.get_current_node()->get_line() << std::endl;

            reader.close();

            return res && have_roads && have_lanes && have_intersections;
        }
        catch(const std::exception &e)
        {
            return false;
        }
    }

    road *network::retreive_road(const str &id)
    {
        std::map<const str, road>::iterator entry(roads.find(id));
        if(entry == roads.end())
            roads.insert(entry, std::make_pair(id, road()));

        return &(roads[id]);
    }

    lane *network::retreive_lane(const str &id)
    {
        std::map<const str, lane>::iterator entry(lanes.find(id));
        if(entry == lanes.end())
            lanes.insert(entry, std::make_pair(id, lane()));

        return &(lanes[id]);
    }

    intersection *network::retreive_intersection(const str &id)
    {
        std::map<const str, intersection>::iterator entry(intersections.find(id));
        if(entry == intersections.end())
            intersections.insert(entry, std::make_pair(id, intersection()));

        return &(intersections[id]);
    }
};
