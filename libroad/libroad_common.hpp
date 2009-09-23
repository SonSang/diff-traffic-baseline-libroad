#ifndef _LIBROAD_COMMON_HPP_
#define _LIBROAD_COMMON_HPP_

#include <vector>
#include <map>
#include <algorithm>
#include <iostream>

#include <tr1/functional>
#include <tr1/unordered_map>

#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/tokenizer.hpp>

#include <glibmm/ustring.h>

#include <tvmet/Vector.h>
#include <tvmet/Matrix.h>

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

typedef tvmet::Vector<double, 2>   vec2d;
typedef tvmet::Vector<float, 2>    vec2f;
typedef vec2f                      intervalf;
typedef tvmet::Vector<float, 3>    vec3f;
typedef tvmet::Matrix<float, 3, 3> mat3x3f;
typedef tvmet::Matrix<float, 4, 4> mat4x4f;
#endif
