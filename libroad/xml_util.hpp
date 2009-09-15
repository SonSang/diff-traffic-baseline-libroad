#ifndef _XML_UTIL_HPP_
#define _XML_UTIL_HPP_

#include <boost/lexical_cast.hpp>
#include <libxml/xmlreader.h>
#include <map>

inline bool is_opening_element(xmlTextReaderPtr reader, const xmlChar *eltname)
{
    const xmlChar *name = xmlTextReaderConstName(reader);

    return name && xmlTextReaderNodeType(reader) == XML_READER_TYPE_ELEMENT
        && xmlStrEqual(eltname, name);
}

inline bool is_closing_element(xmlTextReaderPtr reader, const xmlChar *eltname)
{
    const xmlChar *name = xmlTextReaderConstName(reader);

    return name && xmlTextReaderNodeType(reader) == XML_READER_TYPE_END_ELEMENT
        && xmlStrEqual(eltname, name);
}

template <class closure, typename T>
inline bool read_map(closure &c, std::map<std::string, T> &themap, xmlTextReaderPtr reader, const xmlChar *item_name, const xmlChar *container_name)
{
    int ret;
    do
    {
        ret = xmlTextReaderRead(reader);
        if(ret != 1)
            return false;

        if(xmlTextReaderNodeType(reader) == XML_READER_TYPE_ELEMENT)
        {
            const xmlChar *name = xmlTextReaderConstName(reader);
            if(!name)
                return false;

            if(xmlStrEqual(name, item_name))
            {
                T new_item;
                if(!c.xml_read(new_item, reader))
                    return false;

                themap[new_item.id] = new_item;
            }
            else
                return false;
        }
    }
    while(ret == 1 && !is_closing_element(reader, container_name));

    return ret == 1;
}

inline bool read_leaf_text(std::string &res, xmlTextReaderPtr reader, const char *endtag)
{
    bool read_text = false;

    do
    {
        int ret = xmlTextReaderRead(reader);
        if(ret != 1)
            return false;

        if(xmlTextReaderNodeType(reader) == XML_READER_TYPE_TEXT)
        {
            res.append((const char *) xmlTextReaderConstValue(reader));
            read_text = true;
        }
    }
    while(!is_closing_element(reader, BAD_CAST endtag));

    return read_text;
}
#endif
