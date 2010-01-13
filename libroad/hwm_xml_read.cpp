#include "hwm_network.hpp"
#include "xml_util.hpp"

bool polyline_road::xml_read(xmlpp::TextReader &reader, const vec3f &scale)
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

                    points_.push_back(vec3f(pos*scale));
                }
            }
        }
    }
    while(!is_closing_element(reader, "points"));

    if(!initialize())
        throw std::exception();

    return read_to_close(reader, "line_rep");
}

bool arc_road::xml_read_as_poly(xmlpp::TextReader &reader, const vec3f &scale)
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

                    points_.push_back(vec3f(pos*scale));
                }
            }
        }
    }
    while(!is_closing_element(reader, "points"));

    if(!initialize())
        throw std::exception();

    return read_to_close(reader, "line_rep");
}

template <class T>
template <class C>
bool partition01<T>::xml_read(C &n, xmlpp::TextReader &reader, const str &tag)
{
    if(!read_to_open(reader, "interval"))
        return false;

    if(is_closing_element(reader, "interval"))
        return true;

    if(!(read_to_open(reader, "base") &&
         read_to_open(reader, tag)))
        return false;

    T elt0;
    if(!elt0.xml_read(n, reader))
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
            if(!elt.xml_read(n, reader))
                return false;

            insert(div, elt);

            if(!read_to_close(reader, "divider"))
                return false;
        }
    }

    if(!empty() || !elt0.empty())
        insert(0.0f, elt0);

    return res;
}


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

    bool intersection::state::xml_read(xmlpp::TextReader &reader)
    {
        assert(is_opening_element(reader, "state"));

        bool res = get_attribute(duration, reader, "duration");

        while(res && !is_closing_element(reader, "state"))
        {
            res = read_skip_comment(reader);
            if(is_opening_element(reader, "lane_pair"))
            {
                size_t in_id, out_id;
                if(!(get_attribute(in_id, reader, "in_id") &&
                     get_attribute(out_id, reader, "out_id")))
                    return false;

                out_states[out_id].in_ref = in_id;
                in_states[in_id].out_ref  = out_id;

                res = read_to_close(reader, "lane_pair");
            }
        }

        return res;
    }

    bool lane::road_membership::xml_read(network &n, xmlpp::TextReader &reader)
    {
        assert(is_opening_element(reader, "road_membership"));

        str ref;
        if(!(get_attribute(ref, reader, "parent_road_ref") &&
             get_attribute(interval[0], reader, "interval_start") &&
             get_attribute(interval[1], reader, "interval_end") &&
             get_attribute(lane_position, reader, "lane_position")))
            return false;

        parent_road = retrieve<road>(n.roads, ref);

        return read_to_close(reader, "road_membership");
    }

    bool lane::adjacency::xml_read(network &n, xmlpp::TextReader &reader)
    {
        assert(is_opening_element(reader, "lane_adjacency"));

        str ref;
        int acount = get_attribute(ref, reader, "lane_ref") +
                     get_attribute(neighbor_interval[0], reader, "interval_start") +
                     get_attribute(neighbor_interval[1], reader, "interval_end");
        if(acount == 3)
            neighbor = retrieve<lane>(n.lanes, ref);
        else if(acount == 0)
            neighbor = 0;
        else
            return false;

        return read_to_close(reader, "lane_adjacency");
    }

    bool lane::terminus::xml_read(network &n, const lane *parent, xmlpp::TextReader &reader, const str &tag)
    {
        assert(is_opening_element(reader, "dead_end"));

        return read_to_close(reader, tag);
    }

    bool lane::intersection_terminus::xml_read(network &n, const lane *parent, xmlpp::TextReader &reader, const str &tag)
    {
        assert(is_opening_element(reader, "intersection_ref"));

        str ref;
        if(!(get_attribute(ref, reader, "ref")))
            return false;

        adjacent_intersection = retrieve<intersection>(n.intersections, ref);
        intersect_in_ref = -1;

        if(!adjacent_intersection->id.empty())
        {
            const std::vector<lane*> &cont(tag == "start" ? adjacent_intersection->outgoing : adjacent_intersection->incoming);
            size_t pos = 0;
            while(cont[pos] != parent)
            {
                    if(pos >= cont.size())
                        return false;
                    ++pos;
                }
            intersect_in_ref = pos;
        }

        return read_to_close(reader, tag);
    }

    bool lane::lane_terminus::xml_read(network &n, const lane *parent, xmlpp::TextReader &reader, const str &tag)
    {
        assert(is_opening_element(reader, "lane_ref"));

        str ref;
        if(!(get_attribute(ref, reader, "ref")))
            return false;

        adjacent_lane = retrieve<lane>(n.lanes, ref);

        return read_to_close(reader, tag);
    }

    bool road::xml_read(network &n, xmlpp::TextReader &reader, const vec3f &scale)
    {
        assert(is_opening_element(reader, "road"));

        str read_id;
        get_attribute(read_id, reader, "id");
        if(id != read_id)
            return false;

        if(!get_attribute(name, reader, "name"))
            return false;

        if(!read_to_open(reader, "line_rep"))
            return false;

        if(!rep.xml_read_as_poly(reader, scale))
            return false;

        return read_to_close(reader, "road");
    }

    inline static lane::terminus *terminus_helper(network &n, const lane *parent, xmlpp::TextReader &reader, const str &tag)
    {
        assert(is_opening_element(reader, tag));

        lane::terminus *res = 0;

        while(!res)
        {
            if(!read_skip_comment(reader))
                return 0;

            if(is_opening_element(reader, "dead_end"))
                res = new lane::terminus();
            else if(is_opening_element(reader, "intersection_ref"))
                res = new lane::intersection_terminus();
            else if(is_opening_element(reader, "lane_ref"))
                res = new lane::lane_terminus();
        }

        res->xml_read(n, parent, reader, tag);

        return res;
    }

    bool lane::xml_read(network &n, xmlpp::TextReader &reader)
    {
        assert(is_opening_element(reader, "lane"));

        str read_id;
        get_attribute(read_id, reader, "id");
        if(id != read_id)
            return false;

        if(!get_attribute(speedlimit, reader, "speedlimit"))
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
                res = have_start = (start = terminus_helper(n, this, reader, "start"));
            else if(is_opening_element(reader, "end"))
                res = have_end = (end = terminus_helper(n, this, reader, "end"));
            else if(is_opening_element(reader, "road_intervals"))
            {
                if(!road_memberships.xml_read(n, reader, "road_membership"))
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
                        have_left = (left.xml_read(n, reader, "lane_adjacency") &&
                                     read_to_close(reader, "left"));
                    }
                    else if(is_opening_element(reader, "right"))
                    {
                        have_right = (right.xml_read(n, reader, "lane_adjacency") &&
                                      read_to_close(reader, "right"));
                    }
                }
                have_adjacency = res && have_left && have_right;
            }
        }

        return (res && have_start && have_end && have_road_int &&
                have_adjacency);
    }

    static inline bool xml_incident_read(network &n, std::vector<lane*> &lv, xmlpp::TextReader &reader, bool incoming, const str &intersect_id)
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
            lane::intersection_terminus *term = dynamic_cast<lane::intersection_terminus*>(incoming ? lv[loc]->end : lv[loc]->start);

            if(!term->adjacent_intersection || term->adjacent_intersection->id != intersect_id)
                return false;
            term->intersect_in_ref = loc;
        }

        return read_to_close(reader, "lane_ref");
    }

    bool intersection::xml_read(network &n, xmlpp::TextReader &reader)
    {
        assert(is_opening_element(reader, "intersection"));

        str in_id;
        get_attribute(in_id, reader, "id");
        if(id != in_id)
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
                        res = xml_incident_read(n, incoming, reader, true, id);
                }
            }
            else if(is_opening_element(reader, "outgoing"))
            {
                while(res && !is_closing_element(reader, "outgoing"))
                {
                    res = read_skip_comment(reader);

                    if(is_opening_element(reader, "lane_ref"))
                        res = xml_incident_read(n, outgoing, reader, false, id);
                }
            }
        }

        res = read_to_open(reader, "states");

        while(res && !is_closing_element(reader, "states"))
        {
            res = read_skip_comment(reader);

            if(is_opening_element(reader, "state"))
            {
                size_t read_id;
                if(!get_attribute(read_id, reader, "id"))
                    return false;

                if(states.size() <= read_id)
                    states.resize(read_id+1);

                states[read_id].in_states.resize(incoming.size());
                BOOST_FOREACH(intersection::state::out_id &oid, states[read_id].in_states)
                {
                    oid.out_ref = -1;
                    oid.fict_lane = 0;
                }
                states[read_id].out_states.resize(outgoing.size());
                BOOST_FOREACH(intersection::state::in_id &iid, states[read_id].out_states)
                {
                    iid.in_ref = -1;
                    iid.fict_lane = 0;
                }

                res = states[read_id].xml_read(reader);
            }
        }

        return res;
    }

    network load_xml_network(const char *filename, const vec3f &scale)
    {
        network n;
        n.scale = scale;
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
