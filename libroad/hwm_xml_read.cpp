#include "hwm_network.hpp"
#include "xml_util.hpp"

void polyline_road::xml_read(xmlpp::TextReader &reader, const vec3f &scale)
{
    assert(is_opening_element(reader, "line_rep"));

    read_to_open(reader, "points");

    do
    {
        read_skip_comment(reader);

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
        throw xml_error(reader, "Failed to initialize polyine");

    read_to_close(reader, "line_rep");
}

void arc_road::xml_read(xmlpp::TextReader &reader, const vec3f &scale)
{
    assert(is_opening_element(reader, "arc_line_rep"));

    read_to_open(reader, "points");

    do
    {
        read_skip_comment(reader);

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

    read_to_open(reader, "radii");

    while(!is_closing_element(reader, "radii"))
    {
        read_skip_comment(reader);

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

                    float rad;
                    instr >> rad;

                    radii_.push_back(rad*scale[0]);
                }
            }
        }
    }

    if(!initialize_from_points_radii(points_, radii_))
        throw xml_error(reader, "Failed to initialize arc_road!");

    read_to_close(reader, "arc_line_rep");
}

void arc_road::xml_read_as_poly(xmlpp::TextReader &reader, const vec3f &scale)
{
    assert(is_opening_element(reader, "line_rep"));

    read_to_open(reader, "points");

    do
    {
        read_skip_comment(reader);

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

    if(!initialize_from_polyline(0.7f, points_))
        throw xml_error(reader, "Failed to initialize arc_road");

    read_to_close(reader, "line_rep");
}

template <class T>
template <class C>
void partition01<T>::xml_read(C &n, xmlpp::TextReader &reader, const str &tag)
{
    read_to_open(reader, "interval");

    if(is_closing_element(reader, "interval"))
        return;

    read_to_open(reader, "base") ;
    read_to_open(reader, tag);

    T elt0;
    elt0.xml_read(n, reader);

    read_to_close(reader, "base");
    while(!is_closing_element(reader, "interval"))
    {
        read_skip_comment(reader);

        if(is_opening_element(reader, "divider"))
        {
            float div;
            get_attribute(div, reader, "value");

            read_to_open(reader, tag);

            T elt;
            elt.xml_read(n, reader);

            insert(div, elt);

            read_to_close(reader, "divider");
        }
    }

    if(!empty() || !elt0.empty())
        insert(0.0f, elt0);
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

    void intersection::state::xml_read(xmlpp::TextReader &reader)
    {
        assert(is_opening_element(reader, "state"));

        get_attribute(duration, reader, "duration");

        while(!is_closing_element(reader, "state"))
        {
            read_skip_comment(reader);
            if(is_opening_element(reader, "lane_pair"))
            {
                size_t in_id, out_id;
                get_attribute(in_id, reader, "in_id");
                get_attribute(out_id, reader, "out_id");
                state_pairs.insert(state_pair(in_id, out_id));

                read_to_close(reader, "lane_pair");
            }
        }
    }

    void lane::road_membership::xml_read(network &n, xmlpp::TextReader &reader)
    {
        assert(is_opening_element(reader, "road_membership"));

        str ref;
        get_attribute(ref, reader, "parent_road_ref");
        get_attribute(interval[0], reader, "interval_start");
        get_attribute(interval[1], reader, "interval_end");
        get_attribute(lane_position, reader, "lane_position");

        parent_road = retrieve<road>(n.roads, ref);

        read_to_close(reader, "road_membership");
    }

    void lane::adjacency::xml_read(network &n, xmlpp::TextReader &reader)
    {
        assert(is_opening_element(reader, "lane_adjacency"));

        str ref;
        int acount = 0;
        try
        {
            get_attribute(ref, reader, "lane_ref");
            ++acount;
        }
        catch(missing_attribute &ma)
        {
        }

        try
        {
            get_attribute(neighbor_interval[0], reader, "interval_start");
            ++acount;
        }
        catch(missing_attribute &ma)
        {
        }

        try
        {
            get_attribute(neighbor_interval[1], reader, "interval_end");
            ++acount;
        }
        catch(missing_attribute &ma)
        {
        }

        if(acount == 3)
            neighbor = retrieve<lane>(n.lanes, ref);
        else if(acount == 0)
            neighbor = 0;
        else
            throw xml_error(reader, "Lane adjacency missing attributes");

        read_to_close(reader, "lane_adjacency");
    }

    void lane::terminus::xml_read(network &n, const lane *parent, xmlpp::TextReader &reader, const str &tag)
    {
        assert(is_opening_element(reader, "dead_end"));
        read_to_close(reader, tag);
    }

    void lane::intersection_terminus::xml_read(network &n, const lane *parent, xmlpp::TextReader &reader, const str &tag)
    {
        assert(is_opening_element(reader, "intersection_ref"));

        str ref;
        get_attribute(ref, reader, "ref");

        adjacent_intersection = retrieve<intersection>(n.intersections, ref);
        intersect_in_ref = -1;

        if(!adjacent_intersection->id.empty())
        {
            const std::vector<lane*> &cont(tag == "start" ? adjacent_intersection->outgoing : adjacent_intersection->incoming);
            size_t pos = 0;
            while(cont[pos] != parent)
            {
                if(pos >= cont.size())
                    throw xml_error(reader, boost::str(boost::format("Lane %s reports that it is incident on intersection %s, but intersection does not") % parent->id % adjacent_intersection->id));

                ++pos;
            }
            intersect_in_ref = pos;
        }

        read_to_close(reader, tag);
    }

    void lane::lane_terminus::xml_read(network &n, const lane *parent, xmlpp::TextReader &reader, const str &tag)
    {
        assert(is_opening_element(reader, "lane_ref"));

        str ref;
        get_attribute(ref, reader, "ref");

        adjacent_lane = retrieve<lane>(n.lanes, ref);

        read_to_close(reader, tag);
    }

    void road::xml_read(const vec3f &scale, xmlpp::TextReader &reader)
    {
        assert(is_opening_element(reader, "road"));

        str read_id;
        get_attribute(read_id, reader, "id");
        if(id != read_id)
            throw xml_error(reader, boost::str(boost::format("Id mismatch: had %s, read %s") % id % read_id));

        get_attribute(name, reader, "name");

        while(1)
        {
            read_skip_comment(reader);

            if(is_opening_element(reader, "line_rep"))
            {
                rep.xml_read_as_poly(reader, scale);
                break;
            }
            else if(is_opening_element(reader, "arc_line_rep"))
            {
                rep.xml_read(reader, scale);
                break;
            }
        }

        read_to_close(reader, "road");
    }

    inline static lane::terminus *terminus_helper(network &n, const lane *parent, xmlpp::TextReader &reader, const str &tag)
    {
        assert(is_opening_element(reader, tag));

        lane::terminus *res = 0;

        while(!res)
        {
            read_skip_comment(reader);

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

    void lane::xml_read(network &n, xmlpp::TextReader &reader)
    {
        assert(is_opening_element(reader, "lane"));

        str read_id;
        get_attribute(read_id, reader, "id");
        if(id != read_id)
            throw xml_error(reader, "SNA?");//viboost::str(boost::format("Id mismatch: had %s, read %s") % id % read_id));

        get_attribute(speedlimit, reader, "speedlimit");

        active = true;

        bool have_start     = false;
        bool have_end       = false;
        bool have_road_int  = false;
        bool have_adjacency = false;
        while(!is_closing_element(reader, "lane"))
        {
            read_skip_comment(reader);

            if(is_opening_element(reader, "start"))
            {
                start = terminus_helper(n, this, reader, "start");
                have_start = true;
            }
            else if(is_opening_element(reader, "end"))
            {
                have_end = true;
                end = terminus_helper(n, this, reader, "end");
            }
            else if(is_opening_element(reader, "road_intervals"))
            {
                road_memberships.xml_read(n, reader, "road_membership");
                read_to_close(reader, "road_intervals");
                have_road_int = true;
            }
            else if(is_opening_element(reader, "adjacency_intervals"))
            {
                bool have_left  = false;
                bool have_right = false;
                while(!is_closing_element(reader, "adjacency_intervals"))
                {
                    read_skip_comment(reader);

                    if(is_opening_element(reader, "left"))
                    {
                        left.xml_read(n, reader, "lane_adjacency");
                        have_left = true;
                        read_to_close(reader, "left");
                    }
                    else if(is_opening_element(reader, "right"))
                    {
                        right.xml_read(n, reader, "lane_adjacency");
                        have_right = true;
                        read_to_close(reader, "right");
                    }
                }
                if(!have_left)
                    throw xml_error(reader, boost::str(boost::format("No left adjacency in lane %s!") % id));
                if(!have_right)
                    throw xml_error(reader, boost::str(boost::format("No right adjacency in lane %s!") % id));
                have_adjacency = true;
            }
        }

        if(!have_start)
            throw xml_error(reader, boost::str(boost::format("No start terminus in lane %s!") % id));
        if(!have_end)
            throw xml_error(reader, boost::str(boost::format("No end terminus in lane %s!") % id));
        if(!have_road_int)
            throw xml_error(reader, boost::str(boost::format("No road memberships in lane %s!") % id));
        if(!have_adjacency)
            throw xml_error(reader, boost::str(boost::format("No adjacencies in lane %s!") % id));
    }

    static inline void xml_incident_read(network &n, std::vector<lane*> &lv, xmlpp::TextReader &reader, bool incoming, const str &intersect_id)
    {
        assert(is_opening_element(reader, "lane_ref"));

        str ref;
        size_t loc;
        get_attribute(ref, reader, "ref");
        get_attribute(loc, reader, "local_id");

        if(lv.size() <= loc)
            lv.resize(loc+1);

        assert(lv.size() > loc);
        lv[loc] = retrieve<lane>(n.lanes, ref);

        if(!lv[loc]->id.empty())
        {
            lane::intersection_terminus *term = dynamic_cast<lane::intersection_terminus*>(incoming ? lv[loc]->end : lv[loc]->start);

            if(!term->adjacent_intersection || term->adjacent_intersection->id != intersect_id)
                throw xml_error(reader, boost::str(boost::format("Intersection %s reports that it is incident on lane %s, but lane does not") % intersect_id % lv[loc]->id));
            term->intersect_in_ref = loc;
        }

        read_to_close(reader, "lane_ref");
    }

    void intersection::xml_read(network &n, xmlpp::TextReader &reader)
    {
        assert(is_opening_element(reader, "intersection"));
        assert(id != str());
        str in_id;
        get_attribute(in_id, reader, "id");
        if(id != in_id)
            throw xml_error(reader, boost::str(boost::format("Id mismatch: had %s, read %s") % id % in_id));

        read_to_open(reader, "incident");

        while(!is_closing_element(reader, "incident"))
        {
            read_skip_comment(reader);

            if(is_opening_element(reader, "incoming"))
            {
                while(!is_closing_element(reader, "incoming"))
                {
                    read_skip_comment(reader);

                    if(is_opening_element(reader, "lane_ref"))
                        xml_incident_read(n, incoming, reader, true, id);
                }
            }
            else if(is_opening_element(reader, "outgoing"))
            {
                while(!is_closing_element(reader, "outgoing"))
                {
                    read_skip_comment(reader);

                    if(is_opening_element(reader, "lane_ref"))
                        xml_incident_read(n, outgoing, reader, false, id);
                }
            }
        }

        read_to_open(reader, "states");

        while(!is_closing_element(reader, "states"))
        {
            read_skip_comment(reader);

            if(is_opening_element(reader, "state"))
            {
                size_t read_id;
                get_attribute(read_id, reader, "id");

                if(states.size() <= read_id)
                    states.resize(read_id+1);

                states[read_id].xml_read(reader);
            }
        }
    }

    network load_xml_network(const char *filename, const vec3f &scale)
    {
        network n;
        xmlpp::TextReader reader(filename);

        read_skip_comment(reader);

        if(!is_opening_element(reader, "network"))
            throw xml_error(reader, "No network element found!");

        str version;
        get_attribute(version, reader, "version");
        if(version != "1.3")
            throw xml_error(reader, "Invalid network xml version!");

        get_attribute(n.lane_width, reader, "lane_width");
        if(n.lane_width <= 0.0f)
            throw xml_error(reader, "Invalid lane_width!");

        get_attribute(n.name, reader, "name");

        get_attribute(n.gamma, reader, "gamma");
        if(n.gamma <= 0.0f ||
           n.gamma >= 1.0f)
            throw xml_error(reader, "Invalid gamma!");

        bool have_roads         = false;
        bool have_lanes         = false;
        bool have_intersections = false;
        while(!is_closing_element(reader, "network"))
        {
            read_skip_comment(reader);

            if(is_opening_element(reader, "roads"))
            {
                read_map(scale, n.roads, reader, "road", "roads");
                have_roads = true;
            }
            else if(is_opening_element(reader, "lanes"))
            {
                read_map(n, n.lanes, reader, "lane", "lanes");
                have_lanes = true;
            }
            else if(is_opening_element(reader, "intersections"))
            {
                read_map(n, n.intersections, reader, "intersection", "intersections");
                have_intersections = true;
            }
        }

        reader.close();

        return n;
    }
}
