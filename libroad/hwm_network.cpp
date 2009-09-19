#include "hwm_network.hpp"
#include <iostream>

namespace hwm
{
    bool network::xml_read(road &r, xmlpp::TextReader &reader)
    {
        if(!(get_attribute(r.id,   reader, "id") &&
             get_attribute(r.name, reader, "name")))
            return false;

        if(!read_to_open(reader, "line_rep"))
            return false;

        if(!r.rep.xml_read(reader))
            return false;

        return read_to_close(reader, "line_rep");
    }

    bool network::xml_read(lane &l, xmlpp::TextReader &reader)
    {
        if(!(get_attribute(l.id,         reader, "id") &&
             get_attribute(l.speedlimit, reader, "speedlimit")))
            return false;

        return read_to_close(reader, "lane");
    }

    bool network::xml_read(intersection &i, xmlpp::TextReader &reader)
    {
        if(!get_attribute(i.id, reader, "id"))
             return false;

        return read_to_close(reader, "intersection");
    }

    bool network::xml_read(const char *filename)
    {
        try
        {
            xmlpp::TextReader reader(filename);

            bool res;
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

            do
            {
                res = read_skip_comment(reader);

                if(is_opening_element(reader, "roads"))
                    res = read_map(*this, roads, reader, "road", "roads");
                else if(is_opening_element(reader, "lanes"))
                    res = read_map(*this, lanes, reader, "lane", "lanes");
                else if(is_opening_element(reader, "intersections"))
                    res = read_map(*this, intersections, reader, "intersection", "intersections");
            }
            while(res && !is_closing_element(reader, "network"));

            reader.close();

            return res;
        }
        catch(const std::exception &e)
        {
            return false;
        }
    }
};
