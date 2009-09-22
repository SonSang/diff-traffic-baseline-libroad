#include "hwm_network.hpp"
#include "xml_util.hpp"

namespace hwm
{
    template <class T>
    static inline T* retrieve(typename strhash<T>::type &m, const str &id)
    {
        typedef typename strhash<T>::type val;
        typename strhash<T>::type::iterator entry(m.find(id));
        if(entry == m.end())
            m.insert(entry, std::make_pair(id, T()));

        return &(m[id]);
    }

    static inline bool xml_read(polyline_road &pr, xmlpp::TextReader &reader)
    {
        assert(is_opening_element(reader, "line_rep"));

        if(!read_to_open(reader, "points"))
            return false;

        do
        {
            if(!read_skip_comment(reader))
                return false;

            if(reader.get_node_type() == xmlpp::TextReader::Text ||
               reader.get_node_type() == xmlpp::TextReader::SignificantWhitespace)
            {
                std::string                 res(reader.get_value());
                typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
                boost::char_separator<char> linesep("\n");
                tokenizer                   linetokens(res, linesep);

                BOOST_FOREACH(const std::string &ltok, linetokens)
                {
                    std::string trim_ltok(ltok);
                    boost::trim_left(trim_ltok);
                    if(!trim_ltok.empty())
                    {
                        std::stringstream instr(trim_ltok);

                        vec3f pos;
                        instr >> pos[0];
                        instr >> pos[1];
                        instr >> pos[2];

                        pr.points_.push_back(pos);
                    }
                }
            }
        }
        while(!is_closing_element(reader, "points"));

        pr.initialize();

        return read_to_close(reader, "line_rep");
    }

    static inline bool xml_read(network &n, intersection::state &s, xmlpp::TextReader &reader)
    {
        assert(is_opening_element(reader, "state"));

        bool res = get_attribute(s.duration, reader, "duration");

        while(res && !is_closing_element(reader, "state"))
        {
            res = read_skip_comment(reader);
            if(is_opening_element(reader, "lane_pair"))
            {
                size_t in_id, out_id;
                if(!(get_attribute(in_id, reader, "in_id") &&
                     get_attribute(out_id, reader, "out_id")))
                    return false;

                s.out_states[out_id].in_ref = in_id;
                s.in_states[in_id].out_ref  = out_id;

                res = read_to_close(reader, "lane_pair");
            }
        }

        return res;
    }

    static inline bool xml_read(network &n, std::vector<lane*> &lv, xmlpp::TextReader &reader, bool incoming, const str &intersect_id)
    {
        assert(is_opening_element(reader, "lane_ref"));

        str ref;
        size_t loc;
        if(!(get_attribute(ref, reader, "ref") &&
             get_attribute(loc, reader, "local_id")))
            return false;

        if(lv.size() <= loc)
            lv.resize(loc+1);

        assert(lv.size() > loc);
        lv[loc] = retrieve<lane>(n.lanes, ref);

        if(!lv[loc]->id.empty())
        {
            lane::terminus &term = incoming ? lv[loc]->end : lv[loc]->start;

            if(!term.inters || term.inters->id != intersect_id)
                return false;
            term.intersect_in_ref = loc;
        }

        return read_to_close(reader, "lane_ref");
    }

    template <class T>
    static inline bool xml_read(network &n, partition01<T> &part, xmlpp::TextReader &reader, const str &tag)
    {
        if(!read_to_open(reader, "interval"))
            return false;

        if(is_closing_element(reader, "interval"))
           return true;

        if(!(read_to_open(reader, "base") &&
             read_to_open(reader, tag)))
            return false;

        T elt0;
        if(!xml_read(n, elt0, reader))
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

                T elt;
                if(!xml_read(n, elt, reader))
                    return false;

                part.insert(div, elt);

                if(!read_to_close(reader, "divider"))
                    return false;
            }
        }

        if(!part.empty() || !elt0.empty())
           part.insert(0.0f, elt0);

        return res;
    }

    static inline bool xml_read(network &n, lane::road_membership &rm, xmlpp::TextReader &reader)
    {
        assert(is_opening_element(reader, "road_membership"));

        str ref;
        if(!(get_attribute(ref, reader, "parent_road_ref") &&
             get_attribute(rm.interval[0], reader, "interval_start") &&
             get_attribute(rm.interval[1], reader, "interval_end") &&
             get_attribute(rm.lane_position, reader, "lane_position")))
            return false;

        rm.parent_road = retrieve<road>(n.roads, ref);

        return read_to_close(reader, "road_membership");
    }

    static inline bool xml_read(network &n, lane::adjacency &la, xmlpp::TextReader &reader)
    {
        assert(is_opening_element(reader, "lane_adjacency"));

        str ref;
        int acount = get_attribute(ref, reader, "lane_ref") +
                     get_attribute(la.neighbor_interval[0], reader, "interval_start") +
                     get_attribute(la.neighbor_interval[1], reader, "interval_end");
        if(acount == 3)
            la.neighbor = retrieve<lane>(n.lanes, ref);
        else if(acount == 0)
            la.neighbor = 0;
        else
            return false;

        return read_to_close(reader, "lane_adjacency");
    }

    static inline bool xml_read(network &n, lane::terminus &lt, const lane *parent, xmlpp::TextReader &reader, const str &tag)
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

            lt.inters = retrieve<intersection>(n.intersections, ref);
            lt.intersect_in_ref = -1;

            if(!lt.inters->id.empty())
            {
                const std::vector<lane*> &cont(tag == "start" ? lt.inters->outgoing : lt.inters->incoming);
                size_t pos = 0;
                while(cont[pos] != parent)
                {
                    if(pos >= cont.size())
                        return false;
                    ++pos;
                }
                lt.intersect_in_ref = pos;
            }

            if(!read_to_close(reader, "intersection_ref"))
                return false;
        }

        return read_to_close(reader, tag);
    }

    static inline bool xml_read(network &n, road &r, xmlpp::TextReader &reader)
    {
        assert(is_opening_element(reader, "road"));

        str id;
        get_attribute(id, reader, "id");
        if(id != r.id)
            return false;

        if(!get_attribute(r.name, reader, "name"))
            return false;

        if(!read_to_open(reader, "line_rep"))
            return false;

        if(!xml_read(r.rep, reader))
            return false;

        return read_to_close(reader, "road");
    }

    static inline bool xml_read(network &n, lane &l, xmlpp::TextReader &reader)
    {
        assert(is_opening_element(reader, "lane"));

        str id;
        get_attribute(id, reader, "id");
        if(id != l.id)
            return false;

        if(!get_attribute(l.speedlimit, reader, "speedlimit"))
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
                res = have_start = xml_read(n, l.start, &l, reader, "start");
            else if(is_opening_element(reader, "end"))
                res = have_end = xml_read(n, l.end, &l, reader, "end");
            else if(is_opening_element(reader, "road_intervals"))
            {
                if(!xml_read(n, l.road_memberships, reader, "road_membership"))
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
                        have_left = (xml_read(n, l.left, reader, "lane_adjacency") &&
                                     read_to_close(reader, "left"));
                    }
                    else if(is_opening_element(reader, "right"))
                    {
                        have_right = (xml_read(n, l.right, reader, "lane_adjacency") &&
                                      read_to_close(reader, "right"));
                    }
                }
                have_adjacency = res && have_left && have_right;
            }
        }

        return (res && have_start && have_end && have_road_int &&
                have_adjacency);
    }

    static inline bool xml_read(network &n, intersection &i, xmlpp::TextReader &reader)
    {
        assert(is_opening_element(reader, "intersection"));

        str id;
        get_attribute(id, reader, "id");
        if(id != i.id)
            return false;

        bool res = read_to_open(reader, "incident");

        while(res && !is_closing_element(reader, "incident"))
        {
            res = read_skip_comment(reader);

            if(is_opening_element(reader, "incoming"))
            {
                while(res && !is_closing_element(reader, "incoming"))
                {
                    res = read_skip_comment(reader);

                    if(is_opening_element(reader, "lane_ref"))
                        res = xml_read(n, i.incoming, reader, true, i.id);
                }
            }
            else if(is_opening_element(reader, "outgoing"))
            {
                while(res && !is_closing_element(reader, "outgoing"))
                {
                    res = read_skip_comment(reader);

                    if(is_opening_element(reader, "lane_ref"))
                        res = xml_read(n, i.outgoing, reader, false, i.id);
                }
            }
        }

        res = read_to_open(reader, "states");

        while(res && !is_closing_element(reader, "states"))
        {
            res = read_skip_comment(reader);

            if(is_opening_element(reader, "state"))
            {
                size_t id;
                if(!get_attribute(id, reader, "id"))
                    return false;

                if(i.states.size() <= id)
                    i.states.resize(id+1);

                i.states[id].in_states.resize(i.incoming.size());
                BOOST_FOREACH(intersection::state::out_id &oid, i.states[id].in_states)
                {
                    oid.out_ref = -1;
                    oid.fict_lane = 0;
                }
                i.states[id].out_states.resize(i.outgoing.size());
                BOOST_FOREACH(intersection::state::in_id &iid, i.states[id].out_states)
                {
                    iid.in_ref = -1;
                    iid.fict_lane = 0;
                }

                res = xml_read(n, i.states[id], reader);
            }
        }

        return res;
    }

    network load_xml_network(const char *filename)
    {
        network n;
        xmlpp::TextReader reader(filename);

        if(!read_skip_comment(reader))
            throw std::exception();

        if(!is_opening_element(reader, "network"))
            throw std::exception();

        str version;
        if(!get_attribute(version, reader, "version") ||
           version != "1.2")
            throw std::exception();

        if(!get_attribute(n.name, reader, "name"))
            throw std::exception();

        if(!get_attribute(n.gamma, reader, "gamma") ||
           n.gamma <= 0.0f ||
           n.gamma >= 1.0f)
            throw std::exception();

        bool have_roads         = false;
        bool have_lanes         = false;
        bool have_intersections = false;
        bool res                = true;
        while(res && !is_closing_element(reader, "network"))
        {
            res = read_skip_comment(reader);

            if(is_opening_element(reader, "roads"))
                res = have_roads = read_map(n, n.roads, reader, "road", "roads");
            else if(is_opening_element(reader, "lanes"))
                res = have_lanes = read_map(n, n.lanes, reader, "lane", "lanes");
            else if(is_opening_element(reader, "intersections"))
                res = have_intersections = read_map(n, n.intersections, reader, "intersection", "intersections");
        }

        if(!res)
            std::cerr << "Error at line: " << reader.get_current_node()->get_line() << std::endl;

        reader.close();

        return n;
    }
}
