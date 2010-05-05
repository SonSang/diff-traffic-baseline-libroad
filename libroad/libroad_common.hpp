#ifndef _LIBROAD_COMMON_HPP_
#define _LIBROAD_COMMON_HPP_

const char *libroad_package_string();

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <vector>
#include <map>
#include <algorithm>
#include <iostream>
#include <fstream>

#include <tr1/functional>
#include <tr1/unordered_map>

#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/tokenizer.hpp>
#include <boost/multi_index_container.hpp>
#include <boost/multi_index/hashed_index.hpp>
#include <boost/multi_index/member.hpp>
#include <boost/iostreams/device/file_descriptor.hpp>
#include <boost/iostreams/filtering_stream.hpp>
#include <boost/iostreams/filter/gzip.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/static_assert.hpp>
#include <boost/filesystem.hpp>

#include <glibmm/ustring.h>

#include <tvmet/Vector.h>
#include <tvmet/Matrix.h>
#include <libxml++/libxml++.h>

using std::tr1::hash;

typedef Glib::ustring str;

namespace bf = boost::filesystem;

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

typedef tvmet::Vector<double,      2>    vec2d;
typedef tvmet::Vector<float,       2>    vec2f;
typedef vec2f                            intervalf;
typedef tvmet::Vector<float,       3>    vec3f;
typedef tvmet::Vector<double,      3>    vec3d;
typedef tvmet::Vector<int,         3>    vec3i;
typedef tvmet::Vector<unsigned int,3>    vec3u;
typedef tvmet::Vector<int,         2>    vec2i;
typedef tvmet::Vector<size_t,      2>    vec2u;
typedef tvmet::Vector<float,       4>    vec4f;
typedef tvmet::Matrix<float,       3, 3> mat3x3f;
typedef tvmet::Matrix<float,       4, 4> mat4x4f;

template <typename T>
inline float length2(const T& t1)
{
    return tvmet::dot(t1, t1);
}

template <typename T>
inline float length(const T& t1)
{
    return std::sqrt(length2(t1));
}

template <typename T>
inline float distance2(const T& t1, const T& t2)
{
    const T diff(t2-t1);
    return length2(diff);
}

template <typename T>
inline float distance(const T& t1, const T& t2)
{
    return std::sqrt(distance2(t1, t2));
}

inline float planar_distance(const vec3f &pt1, const vec3f &pt2)
{
    const vec2f diff(pt2[0] - pt1[0], pt2[1] - pt1[1]);
    return std::sqrt(tvmet::dot(diff, diff));
}

template <int B, int E>
struct sub
{
    BOOST_STATIC_ASSERT(B >= 0);
    enum { RESDIM = E - B};

    template <typename T>
    static inline tvmet::Vector<typename T::value_type, RESDIM> &vector(T &src)
    {
        BOOST_STATIC_ASSERT(E <= T::Size);
        return *reinterpret_cast<tvmet::Vector<typename T::value_type, RESDIM>*>(src.data()+B);
    }

    template <typename T>
    static inline const tvmet::Vector<typename T::value_type, RESDIM> &vector(const T &src)
    {
        BOOST_STATIC_ASSERT(E <= T::Size);
        return *reinterpret_cast<const tvmet::Vector<typename T::value_type, RESDIM>*>(src.data()+B);
    }
};

template <typename T, int MAXDIM, int DIM>
inline tvmet::Vector<T, MAXDIM> basis()
{
    BOOST_STATIC_ASSERT(DIM < MAXDIM);
    tvmet::Vector<T, MAXDIM> res;
    sub<0, DIM>::vector(res)        = 0;
    res[DIM]                        = 1;
    sub<DIM+1, MAXDIM>::vector(res) = 0;
    return res;
}

template <typename T, int DIM>
inline T basis()
{
    BOOST_STATIC_ASSERT(DIM < T::Size);
    T res;
    sub<0, DIM>::vector(res)         = 0;
    res[DIM]                         = 1;
    sub<DIM+1, T::Size>::vector(res) = 0;
    return res;
}

template <typename M, typename V>
inline V transform(const M &mat, const V &v)
{
    return sub<0, 3>::vector(vec4f(mat*vec4f(v[0], v[1], v[2], 1.0)));
}

inline boost::iostreams::filtering_ostream *compressing_ostream(const str &filename)
{
    boost::iostreams::filtering_ostream *out_stream = new boost::iostreams::filtering_ostream();
    out_stream->push(boost::iostreams::gzip_compressor(boost::iostreams::gzip_params(9)));
    str zip_name;
    if(filename.substr(static_cast<int>(filename.size())-1, static_cast<int>(filename.size())-1) == str("z"))
        zip_name = filename;
    else
        zip_name = boost::str(boost::format("%s.gz") % filename);
    out_stream->push(boost::iostreams::file_descriptor_sink(zip_name.raw()));
    return out_stream;
}

#endif
