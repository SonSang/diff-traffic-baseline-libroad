#ifndef _XML_UTIL_HPP_
#define _XML_UTIL_HPP_

#include "libroad_common.hpp"
#include <boost/lexical_cast.hpp>
#include <libxml++/libxml++.h>
#include <libxml++/parsers/textreader.h>

inline bool read_skip_comment(xmlpp::TextReader &reader)
{
    bool res;
    do
    {
        res = reader.read();
    }
    while(res && reader.get_node_type() == xmlpp::TextReader::Comment);

    return res;
}

template <typename T>
inline bool get_attribute(T &res, xmlpp::TextReader &reader, const str &eltname)
{
    str val(reader.get_attribute(eltname));

    if(val.empty())
        return false;
    res = boost::lexical_cast<T>(val);
    return true;
}

template <>
inline bool get_attribute(str &res, xmlpp::TextReader &reader, const str &eltname)
{
    res = reader.get_attribute(eltname);
    return !res.empty();
}

inline bool is_opening_element(const xmlpp::TextReader &reader, const str &name)
{
    return (reader.get_node_type() == xmlpp::TextReader::Element
            && reader.get_name() == name);
}

inline bool is_closing_element(const xmlpp::TextReader &reader, const str &name)
{
    return ((reader.get_node_type() == xmlpp::TextReader::EndElement ||
             reader.is_empty_element()) &&
            reader.get_name() == name);
}

inline bool read_to_open(xmlpp::TextReader &reader, const str &opentag)
{
    bool res = true;
    while(res && !is_opening_element(reader, opentag))
        res = read_skip_comment(reader);

    return res;
}

inline bool read_to_close(xmlpp::TextReader &reader, const str &endtag)
{
    bool res = true;
    while(res && !is_closing_element(reader, endtag))
        res = read_skip_comment(reader);

    return res;
}

template <class closure, typename T>
inline bool read_map(closure &c, T &themap, xmlpp::TextReader &reader, const str &item_name, const str &container_name)
{
    bool ret;
    do
    {
        ret = read_skip_comment(reader);
        if(!ret)
            return false;

        if(reader.get_node_type() == xmlpp::TextReader::Element)
        {
            if(reader.get_name() == item_name)
            {
                str id(reader.get_attribute("id"));

                typename T::iterator vp(themap.find(id));
                if(vp == themap.end())
                    vp = themap.insert(vp, std::make_pair(id, typename T::value_type::second_type()));
                vp->second.id = vp->first;

                if(!themap[id].xml_read(c, reader))
                    return false;
            }
            else
                return false;
        }
    }
    while(ret && !is_closing_element(reader, container_name));

    return ret;
}

template <class closure, typename T>
inline bool sumo_read_map(closure &c, T &themap, xmlpp::TextReader &reader, const str &item_name, const str &container_name)
{
    bool ret;
    do
    {
        ret = read_skip_comment(reader);
        if(!ret)
            return false;

        if(reader.get_node_type() == xmlpp::TextReader::Element)
        {
            if(reader.get_name() == item_name)
            {
                str id(reader.get_attribute("id"));

                typename T::iterator vp(themap.find(id));
                if(vp == themap.end())
                    vp = themap.insert(vp, std::make_pair(id, typename T::value_type::second_type()));
                vp->second.id = vp->first;

                if(!xml_read(c, themap[id], reader))
                    return false;
            }
            else
                return false;
        }
    }
    while(ret && !is_closing_element(reader, container_name));

    return ret;
}
#endif
