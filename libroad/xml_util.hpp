#ifndef _XML_UTIL_HPP_
#define _XML_UTIL_HPP_

#include "libroad_common.hpp"
#include <boost/lexical_cast.hpp>
#include <libxml++/libxml++.h>
#include <libxml++/parsers/textreader.h>

//TODO
#include <iostream>

inline str xml_line_str(const xmlpp::Node *n)
{
    if(n)
        return boost::str(boost::format("%d") % n->get_line());
    else
        return str("?");
}

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

struct missing_attribute : std::exception
{
    missing_attribute(const xmlpp::TextReader &r, const str &en) : std::exception(), reader(r), eltname(en)
    {}

    virtual const char *what() const throw()
    {
        return boost::str(boost::format("Line %s: no attribute %s") % xml_line_str(reader.get_current_node()) % eltname).c_str();
    }

    const xmlpp::TextReader &reader;
    const str &eltname;
};

template <typename T>
inline void get_attribute(T &res, xmlpp::TextReader &reader, const str &eltname)
{
    str val(reader.get_attribute(eltname));

    if(val.empty())
        throw missing_attribute(reader, eltname);

    res = boost::lexical_cast<T>(val);
}

template <>
inline void get_attribute(str &res, xmlpp::TextReader &reader, const str &eltname)
{
    res = reader.get_attribute(eltname);

    if(res.empty())
        throw missing_attribute(reader, eltname);
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

struct xml_eof_opening : public std::exception
{
    xml_eof_opening(const xmlpp::Node *b, const str &o) : std::exception(), begin(b), opening(o)
    {
    }

    virtual const char *what() const throw()
    {
        return boost::str(boost::format("Reached EOF while looking for opening %s element! (Started search at line: %s") % opening % xml_line_str(begin)).c_str();
    }

    const xmlpp::Node *begin;
    const str &opening;
};

inline void read_to_open(xmlpp::TextReader &reader, const str &opentag)
{
    const xmlpp::Node *begin = reader.get_current_node();
    while(!is_opening_element(reader, opentag))
        if(!read_skip_comment(reader))
            throw xml_eof_opening(begin, opentag);
}

struct xml_eof_closing : public std::exception
{
    xml_eof_closing(const xmlpp::Node *b, const str &c) : std::exception(), begin(b), closing(c)
    {
    }

    virtual const char *what() const throw()
    {
        return boost::str(boost::format("Reached EOF while looking for closing %s element! (Started search at line: %s") % closing % xml_line_str(begin)).c_str();
    }

    const xmlpp::Node *begin;
    const str         &closing;
};

inline void read_to_close(xmlpp::TextReader &reader, const str &endtag)
{
    const xmlpp::Node *begin = reader.get_current_node();
    while(!is_closing_element(reader, endtag))
        if(!read_skip_comment(reader))
            throw xml_eof_closing(begin, endtag);
}

template <class closure, typename T>
inline bool read_map_no_container_and_children(closure &c, T &themap, xmlpp::TextReader &reader, const str &item_name, const str &child_name)
{
    bool ret;
    do
    {
        ret = read_skip_comment(reader);
        if(!ret)
            return 1;

        if(reader.get_node_type() == xmlpp::TextReader::Element)
        {

            if(reader.get_name() == item_name)
            {
                str id(reader.get_attribute("id"));


                typename T::iterator vp(themap.find(id));
                if(vp == themap.end())
                    vp = themap.insert(vp, std::make_pair(id, typename T::value_type::second_type()));
                vp->second.id = vp->first;

                xml_read(c, themap[id], id, reader);
             }
        }
    }
    while(ret);

    return ret;
}


template <class closure, typename T>
inline bool read_map_no_container(closure &c, T &themap, xmlpp::TextReader &reader, const str &item_name, const str &container_name)
{
    bool ret;
    do
    {
        ret = read_skip_comment(reader);
        if(!ret)
            return 1;

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
        }
    }
    while(ret && !is_closing_element(reader, container_name));

    return ret;
}

template <class closure, typename T>
inline bool read_map(closure &c, T &themap, xmlpp::TextReader &reader, const str &item_name, const str &container_name)
{
    bool ret = true;
    while(ret && !is_closing_element(reader, container_name))
    {
        ret = read_skip_comment(reader);
        if(!ret)
            return false;

        if(reader.get_node_type() == xmlpp::TextReader::Element)
        {
            if(reader.get_name() == item_name)
            {
                str id(reader.get_attribute("id"));
                assert(id != str());
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
