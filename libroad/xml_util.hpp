#ifndef _XML_UTIL_HPP_
#define _XML_UTIL_HPP_

#include "libroad_common.hpp"
#include <boost/lexical_cast.hpp>
#include <libxml++/libxml++.h>
#include <libxml++/parsers/textreader.h>

//TODO
#include <iostream>

inline int xml_line(const xmlpp::Node *n)
{
    if(n)
        return n->get_line();
    else
        return -1;
}

struct xml_error : public std::exception
{
    xml_error(const xmlpp::TextReader &reader, const str &e) : std::exception(), line(xml_line(reader.get_current_node())), error(e)
    {
    }

    virtual const char *what() const throw()
    {
        return boost::str(boost::format("Line %d: %s") % line % error).c_str();
    }

    const int  line;
    const str &error;
};

struct xml_eof : public std::exception
{
    xml_eof(const int l) : std::exception(), line(l)
    {
    }

    virtual const char *what() const throw()
    {
        return boost::str(boost::format("Reached unexpected EOF (started search at line: %d)") % line).c_str();
    }

    const int line;
};

inline void read_skip_comment(xmlpp::TextReader &reader)
{
    const int line = xml_line(reader.get_current_node());

    do
    {
        if(!reader.read())
            throw xml_eof(line);
    }
    while(reader.get_node_type() == xmlpp::TextReader::Comment);
}

struct missing_attribute : std::exception
{
    missing_attribute(const xmlpp::TextReader &r, const str &en) : std::exception(), reader(r), eltname(en)
    {}

    virtual const char *what() const throw()
    {
        return boost::str(boost::format("Line %d: no attribute %s") % xml_line(reader.get_current_node()) % eltname).c_str();
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
    xml_eof_opening(const int l, const str &o) : std::exception(), line(l), opening(o)
    {
    }

    virtual const char *what() const throw()
    {
        return boost::str(boost::format("Reached EOF while looking for opening %s element! (Started search at line: %d)") % opening % line).c_str();
    }

    const int line;
    const str &opening;
};

inline void read_to_open(xmlpp::TextReader &reader, const str &opentag)
{
    const int line = xml_line(reader.get_current_node());

    try
    {
        while(!is_opening_element(reader, opentag))
            read_skip_comment(reader);
    }
    catch(xml_eof &e)
    {
        throw xml_eof_opening(line, opentag);
    }
}

struct xml_eof_closing : public std::exception
{
    xml_eof_closing(const int l, const str &c) : std::exception(), line(l), closing(c)
    {
    }

    virtual const char *what() const throw()
    {
        return boost::str(boost::format("Reached EOF while looking for closing %s element! (Started search at line: %d)") % closing % line).c_str();
    }

    const int  line;
    const str &closing;
};

inline void read_to_close(xmlpp::TextReader &reader, const str &endtag)
{
    const int line = xml_line(reader.get_current_node());

    try
    {
        while(!is_closing_element(reader, endtag))
            read_skip_comment(reader);
    }
    catch(xml_eof &e)
    {
        throw xml_eof_closing(line, endtag);
    }
}

template <class closure, typename T>
inline void read_map_no_container_and_children(closure &c, T &themap, xmlpp::TextReader &reader, const str &item_name, const str &child_name)
{
    do
    {
        read_skip_comment(reader);

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
    while(1);
}


template <class closure, typename T>
inline void read_map_no_container(closure &c, T &themap, xmlpp::TextReader &reader, const str &item_name, const str &container_name)
{
    do
    {
        read_skip_comment(reader);

        if(reader.get_node_type() == xmlpp::TextReader::Element)
        {
            if(reader.get_name() == item_name)
            {
                str id(reader.get_attribute("id"));

                typename T::iterator vp(themap.find(id));
                if(vp == themap.end())
                    vp = themap.insert(vp, std::make_pair(id, typename T::value_type::second_type()));
                vp->second.id = vp->first;

                xml_read(c, themap[id], reader);
            }
        }
    }
    while(!is_closing_element(reader, container_name));
}

template <class closure, typename T>
inline void read_map(closure &c, T &themap, xmlpp::TextReader &reader, const str &item_name, const str &container_name)
{
    while(!is_closing_element(reader, container_name))
    {
        read_skip_comment(reader);

        if(reader.get_node_type() == xmlpp::TextReader::Element)
        {
            if(reader.get_name() != item_name)
                throw xml_error(reader, boost::str(boost::format("Found stray %s in %s container search (expected %s)") % reader.get_name() % container_name % item_name));

            str id(reader.get_attribute("id"));
            assert(id != str());
            typename T::iterator vp(themap.find(id));
            if(vp == themap.end())
                vp = themap.insert(vp, std::make_pair(id, typename T::value_type::second_type()));
            vp->second.id = vp->first;

            themap[id].xml_read(c, reader);
        }
    }
}

template <class closure, typename T>
inline void sumo_read_map(closure &c, T &themap, xmlpp::TextReader &reader, const str &item_name, const str &container_name)
{
    do
    {
        read_skip_comment(reader);

        if(reader.get_node_type() == xmlpp::TextReader::Element)
        {
            if(reader.get_name() != item_name)
                throw xml_error(reader, boost::str(boost::format("Found stray %s in %s container search (expected %s)") % reader.get_name() % container_name % item_name));

            str id(reader.get_attribute("id"));

            typename T::iterator vp(themap.find(id));
            if(vp == themap.end())
                vp = themap.insert(vp, std::make_pair(id, typename T::value_type::second_type()));
            vp->second.id = vp->first;

            xml_read(c, themap[id], reader);
        }
    }
    while(!is_closing_element(reader, container_name));
}
#endif
