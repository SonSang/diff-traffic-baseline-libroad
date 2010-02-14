#include "libroad_common.hpp"

struct path_element
{
    virtual void   reverse()                   = 0;
    virtual vec3f &point0()                    = 0;
    virtual vec3f &point1()                    = 0;
    virtual str    stringify(bool start) const = 0;
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

    virtual str stringify(bool start) const
    {
        const float offset_val = orientation ? -offset : offset;

        if(start)
            return boost::str(boost::format("M%f,%f A%f,%f 0 0,%d %f,%f ") % points[0][0] % points[0][1] % (radius+offset_val) % (radius+offset_val) % orientation % points[1][0] % points[1][1]);
        else
            return boost::str(boost::format("A%f,%f 0 0,%d %f,%f ") %  (radius+offset_val) % (radius+offset_val) % orientation % points[1][0] % points[1][1]);
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
        elements.push_back(new line_segment(p0, p1));
    }

    void add_arc(const vec3f &p0, const float r, const float off, const bool o, const vec3f &p1)
    {
        elements.push_back(new arc_segment(p0, r, off, o, p1));
    }

    void remove_duplicates()
    {
        std::vector<path_element*> new_elements;
        new_elements.push_back(elements.front());
        std::vector<path_element*>::iterator c = boost::next(elements.begin());
        for(; c != elements.end(); ++c)
        {
            if(distance2((*c)->point0(), (new_elements.back())->point0()) < 1e-4 ||
               distance2((*c)->point1(), (new_elements.back())->point1()) < 1e-4)
                delete *c;
            else
                new_elements.push_back(*c);
        }

        elements.swap(new_elements);
    }

    void reverse()
    {
        std::reverse(elements.begin(), elements.end());
        BOOST_FOREACH(path_element *p, elements)
        {
            p->reverse();
        }
    }

    str stringify() const
    {
        str res;
        std::vector<path_element*>::const_iterator c = elements.begin();
        res.append((*c)->stringify(true));
        for(; c != elements.end(); ++c)
            res.append((*c)->stringify(false));

        return res;
    }

    std::vector<path_element*> elements;
};
