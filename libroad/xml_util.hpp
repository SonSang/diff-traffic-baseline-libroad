#ifndef _XML_UTIL_HPP_
#define _XML_UTIL_HPP_

#include <boost/lexical_cast.hpp>
#include <glibmm/ustring.h>
#include <libxml++/libxml++.h>
#include <libxml++/parsers/textreader.h>
#include <map>
#include <iostream>
#include <tr1/functional>
#include <tr1/unordered_map>

using std::tr1::hash;

typedef Glib::ustring str;

namespace std
{
    namespace tr1
    {
        template <>
        struct hash<const str>
        {
            size_t operator()(const Glib::ustring &str) const
            {
                hash<const char*> h;
                return h(str.c_str());
            }
        };
    }
}

template <class T>
struct strhash
{
    typedef std::map<const str, T> type;
};

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
                typename T::value_type::second_type new_item;
                if(!c.xml_read(new_item, reader))
                    return false;

                themap[new_item.id] = new_item;
            }
            else
                return false;
        }
    }
    while(ret && !is_closing_element(reader, container_name));

    return ret;
}
#endif
