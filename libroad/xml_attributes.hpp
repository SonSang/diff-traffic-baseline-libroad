#ifndef _XML_ATTRIBUTES_HPP_
#define _XML_ATTRIBUTES_HPP_

#include <boost/fusion/container/vector.hpp>
#include <boost/fusion/include/vector.hpp>
#include <boost/fusion/sequence/intrinsic/at.hpp>
#include <boost/fusion/container/generation/make_vector.hpp>
#include <boost/fusion/include/make_vector.hpp>
#include <boost/lexical_cast.hpp>

#include <cstring>
#include <libxml/xmlreader.h>

struct xa
{
    typedef enum {NEEDED, OPTIONAL} type;
};

template <typename T>
struct list_matcher
{
    list_matcher(const char *n, T &v, xa::type xt = xa::NEEDED)
        : name(n), val(v), optional(xt == xa::OPTIONAL), found(false)
    {}

    const char *name;

    T &val;
    bool optional;
    bool found;

    inline bool match(const char *nstr, const char *vstr)
    {
        if(!found && strcmp(nstr, name) == 0)
        {
            val = boost::lexical_cast<T>(vstr);
            found = true;
            return true;
        }
        return false;
    }
};

template <typename T, int N>
struct walker
{
    static inline bool walk(T &v, const char *nstr, const char *vstr)
    {
        if(boost::fusion::at_c<N>(v).match(nstr, vstr))
            return true;
        else
            return walker<T, N-1>::walk(v, nstr, vstr);
    }

    static inline bool valid(T &v)
    {
        if(boost::fusion::at_c<N>(v).found || boost::fusion::at_c<N>(v).optional)
            return walker<T, N-1>::valid(v);

        return false;
    }
};

template <typename T>
struct walker<T, 0>
{
    static inline bool walk(T &v, const char *nstr, const char *vstr)
    {
        return boost::fusion::at_c<0>(v).match(nstr, vstr);
    }

    static inline bool valid(T & v)
    {
        return boost::fusion::at_c<0>(v).found || boost::fusion::at_c<0>(v).optional;
    }
};

template <typename T>
bool walk(T &v, const char *nstr, const char *vstr)
{
    return walker<T, boost::fusion::result_of::size<T>::value-1>::walk(v, nstr, vstr);
}

template <typename T>
bool valid(T &v)
{
    return walker<T, boost::fusion::result_of::size<T>::value-1>::valid(v);
}

template <typename T>
inline bool read_attributes(T v, xmlTextReaderPtr reader)
{
    if(xmlTextReaderMoveToFirstAttribute(reader) != 1)
        return false;

    while(1)
    {
        walk(v, (const char*)xmlTextReaderConstName(reader), (const char*)xmlTextReaderConstValue(reader));
        if(xmlTextReaderMoveToNextAttribute(reader) != 1)
            break;
    }

    xmlTextReaderMoveToElement(reader);

    return valid(v);
}

template <class T0>
boost::fusion::vector<list_matcher<T0> > att_list(const char *id0, T0 &elt0, xa::type xt0)
{
    return boost::fusion::make_vector(list_matcher<T0>(id0, elt0, xt0));
}


template <class T0, class T1>
boost::fusion::vector<list_matcher<T0>, list_matcher<T1> > att_list(const char *id0, T0 &elt0, xa::type xt0,
                                                                    const char *id1, T1 &elt1, xa::type xt1)
{
    return boost::fusion::make_vector(list_matcher<T0>(id0, elt0, xt0),
                                      list_matcher<T1>(id1, elt1, xt1));
}

template <class T0, class T1, class T2>
boost::fusion::vector<list_matcher<T0>, list_matcher<T1>, list_matcher<T2> > att_list(const char *id0, T0 &elt0, xa::type xt0,
                                                                                      const char *id1, T1 &elt1, xa::type xt1,
                                                                                      const char *id2, T2 &elt2, xa::type xt2)
{
    return boost::fusion::make_vector(list_matcher<T0>(id0, elt0, xt0),
                                      list_matcher<T1>(id1, elt1, xt1),
                                      list_matcher<T2>(id2, elt2, xt2));

}

template <class T0, class T1, class T2, class T3>
boost::fusion::vector<list_matcher<T0>, list_matcher<T1>, list_matcher<T2>, list_matcher<T3> > att_list(const char *id0, T0 &elt0, xa::type xt0,
                                                                                                        const char *id1, T1 &elt1, xa::type xt1,
                                                                                                        const char *id2, T2 &elt2, xa::type xt2,
                                                                                                        const char *id3, T3 &elt3, xa::type xt3)
{
    return boost::fusion::make_vector(list_matcher<T0>(id0, elt0, xt0),
                                      list_matcher<T1>(id1, elt1, xt1),
                                      list_matcher<T2>(id2, elt2, xt2),
                                      list_matcher<T3>(id3, elt3, xt3));
}
#endif
