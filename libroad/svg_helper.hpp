#ifndef _SVG_HELPER_HPP_
#define _SVG_HELPER_HPP_

#include "libroad_common.hpp"
#if HAVE_CAIRO
#include <cairo.h>
#endif

struct endpoint_param
{
    vec2f p0;
    vec2f p1;
    vec2f radii;
    bool  fa;
    bool  fs;
};

static float angle_func(const vec2f &u, const vec2f &v)
{
    const float sign = copysign(1.0f, u[0]*v[1] - u[1]*v[0]);
    return sign*std::acos(tvmet::dot(u, v)/(length(u)*length(v)));
}

struct center_param
{
    void from_endpoint(const endpoint_param &ep)
    {
        radii = ep.radii;
        phi   = 0;
        const vec2f xp((ep.p0 - ep.p1)/2);

        const float sign = (ep.fa != ep.fs) ? 1.0 : -1.0f;
        const float r02  = radii[0]*radii[0];
        const float r12  = radii[1]*radii[1];
        const float fac  = sign*std::sqrt((r02*r12 -
                                          r02*xp[1]*xp[1] -
                                          r12*xp[0]*xp[0])/
                                         (r02*xp[1]*xp[1] +
                                          r12*xp[0]*xp[0]));
        const vec2f cp(fac*vec2f(radii[0]*xp[1]/radii[1],
                                 -radii[1]*xp[0]/radii[0]));
        center           = vec2f(cp + (ep.p0 + ep.p1)/2);

        theta  = angle_func(vec2f(1, 0), vec2f((xp[0] - cp[0])/radii[0],
                                               (xp[1] - cp[1])/radii[1]));
        dtheta = angle_func(vec2f((xp[0] - cp[0])/radii[0],
                                  (xp[1] - cp[1])/radii[1]),
                            vec2f((-xp[0] - cp[0])/radii[0],
                                  (-xp[1] - cp[1])/radii[1]));
        if(!ep.fs && dtheta > 0)
            dtheta -= M_2_PI;
        else if(ep.fs && dtheta < 0)
            dtheta += M_2_PI;
    }

    vec2f center;
    vec2f radii;
    float phi;
    float theta;
    float dtheta;
};

struct path_element
{
    virtual void          reverse()                    = 0;
    virtual vec3f        &point0()                     = 0;
    virtual vec3f        &point1()                     = 0;
    virtual path_element *copy() const                 = 0;
#if HAVE_CAIRO
    virtual void          cairo_draw(cairo_t *c,
                                     bool start) const = 0;
#endif
    virtual str           stringify(bool start) const  = 0;
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

#if HAVE_CAIRO
    virtual void cairo_draw(cairo_t *c, bool start) const
    {
        if(start)
            cairo_move_to(c, points[0][0], points[0][1]);
        else
            cairo_line_to(c, points[0][0], points[0][1]);
        cairo_line_to(c, points[1][0], points[1][1]);
    }
#endif

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

#if HAVE_CAIRO
    virtual void cairo_draw(cairo_t *c, bool start) const
    {
        const float offset_val = orientation ? -offset : offset;

        if(start)
            cairo_move_to(c, points[0][0], points[0][1]);

        endpoint_param ep;
        ep.radii = vec2f(radius+offset_val, radius+offset_val);
        ep.p0    = sub<0,2>::vector(points[0]);
        ep.p1    = sub<0,2>::vector(points[1]);
        ep.fa    = 0;
        ep.fs    = orientation;

        center_param cp;
        cp.from_endpoint(ep);

        if(cp.dtheta > 0)
            cairo_arc(c, cp.center[0], cp.center[1],
                      cp.radii[0], cp.theta, cp.theta + cp.dtheta);
        else
            cairo_arc_negative(c, cp.center[0], cp.center[1],
                               cp.radii[0], cp.theta, cp.theta + cp.dtheta);
    }
#endif

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

    void cairo_draw(cairo_t *ct, const bool new_path=true) const
    {
        std::vector<path_element*>::const_iterator c = elements.begin();
        (*c)->cairo_draw(ct, new_path);
        ++c;
        for(; c != elements.end(); ++c)
            (*c)->cairo_draw(ct, false);
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
