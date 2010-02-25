#ifndef _SVG_HELPER_HPP_
#define _SVG_HELPER_HPP_

#include "libroad_common.hpp"

struct path_element
{
    virtual void          reverse()                   = 0;
    virtual vec3f        &point0()                    = 0;
    virtual vec3f        &point1()                    = 0;
    virtual path_element *copy() const                = 0;
    virtual str           stringify(bool start) const = 0;
};

struct line_segment : public path_element
{
    line_segment(const vec3f &p0, const vec3f &p1)
    {
        points[0] = p0;
        points[1] = p1;
    }

    virtual void reverse()
    {
        std::swap(points[0], points[1]);
    }

    virtual vec3f &point0()
    {
        return points[0];
    }

    virtual vec3f &point1()
    {
        return points[1];
    }

    virtual path_element *copy() const
    {
        return new line_segment(points[0], points[1]);
    }

    virtual str stringify(bool start) const
    {
        if(start)
            return boost::str(boost::format("M%f,%f L%f,%f ") % points[0][0] % points[0][1] % points[1][0] % points[1][1]);
        else
            return boost::str(boost::format("L%f,%f ") % points[1][0] % points[1][1]);
    }

    vec3f points[2];
};

struct arc_segment : public path_element
{
    arc_segment(const vec3f &p0, const float r, const float off, const bool o, const vec3f &p1)
    {
        points[0]   = p0;
        points[1]   = p1;
        offset      = off;
        orientation = o;
        radius      = r;
    }

    virtual void reverse()
    {
        std::swap(points[0], points[1]);
        orientation = !orientation;
        offset *= -1;
    }

    virtual vec3f &point0()
    {
        return points[0];
    }

    virtual vec3f &point1()
    {
        return points[1];
    }

    virtual path_element *copy() const
    {
        return new arc_segment(points[0], radius, offset, orientation, points[1]);
    }

    virtual str stringify(bool start) const
    {
        const float offset_val = orientation ? -offset : offset;

        if(start)
            return boost::str(boost::format("M%f,%f A%f,%f 0 0,%d %f,%f ") % points[0][0] % points[0][1] % (radius+offset_val) % (radius+offset_val) % orientation % points[1][0] % points[1][1]);
        else
            return boost::str(boost::format("A%f,%f 0 0,%d %f,%f ") % (radius+offset_val) % (radius+offset_val) % orientation % points[1][0] % points[1][1]);
    }

    vec3f points[2];
    float offset;
    bool  orientation;
    float radius;
};

struct path
{
    path() {};
    ~path()
    {
        BOOST_FOREACH(path_element *p, elements)
        {
            delete p;
        }
    }

    void add_line(const vec3f &p0, const vec3f &p1)
    {
        if(distance2(p0, p1) > 1e-6 && (elements.empty() || distance2(elements.back()->point1(), p1) > 1e-6))
            elements.push_back(new line_segment(p0, p1));
    }

    void add_arc(const vec3f &p0, const float r, const float off, const bool o, const vec3f &p1)
    {
        if(distance2(p0, p1) > 1e-6 && (elements.empty() || distance2(elements.back()->point1(), p1) > 1e-6))
            elements.push_back(new arc_segment(p0, r, off, o, p1));
    }

    void reverse()
    {
        std::reverse(elements.begin(), elements.end());
        BOOST_FOREACH(path_element *p, elements)
        {
            p->reverse();
        }
    }

    void append(const path &o)
    {
        if(o.elements.empty())
        return;
        if(!elements.empty())
            add_line(elements.back()->point1(), o.elements.front()->point0());

        BOOST_FOREACH(const path_element *pe, o.elements)
        {
            elements.push_back(pe->copy());
        }
    }

    str stringify() const
    {
        str res;
        std::vector<path_element*>::const_iterator c = elements.begin();
        res.append((*c)->stringify(true));
        ++c;
        for(; c != elements.end(); ++c)
            res.append((*c)->stringify(false));

        return res;
    }

    std::vector<path_element*> elements;
};
#endif
