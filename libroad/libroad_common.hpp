#ifndef _LIBROAD_COMMON_HPP_
#define _LIBROAD_COMMON_HPP_

#include <glibmm/ustring.h>
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

#endif
