#include "libroad/libroad_common.hpp"
#include <GL/glew.h>
#include <FL/Fl.H>
#include <FL/Fl_Gl_Window.H>
#include <FL/Fl_Menu_Button.H>
#include <FL/gl.h>
#include <FL/glu.h>
#include <FL/glut.h>
#include "libroad/hwm_network.hpp"
#include "libroad/geometric.hpp"

struct circle_partition
{
    void insert(float f, bool inside)
    {
        divisions.insert(std::make_pair(f, inside));
    };

    void back_up(std::map<float,bool>::iterator &it)
    {
        if(it == divisions.begin())
            it = divisions.end();
        --it;
    }

    void advance(std::map<float,bool>::iterator &it)
    {
        ++it;
        if(it == divisions.end())
            it = divisions.begin();
    }

    std::map<float,bool> divisions;
};

static vec2f interval_intersection(const vec2f &a, const vec2f &b)
{
    return vec2f(std::max(a[0], b[0]), std::min(a[1], b[1]));
}

static bool interval_okay(const vec2f &a)
{
    return a[0] < a[1];
}

static std::vector<vec2f> interval_overlap(const vec2f &a, const vec2f &b)
{
    std::vector<vec2f> a_pairs;
    if(a[0] > a[1])
    {
        a_pairs.push_back(vec2f(0, a[1]));
        a_pairs.push_back(vec2f(a[0], 2*M_PI));
    }
    else
        a_pairs.push_back(a);
    std::vector<vec2f> b_pairs;
    if(b[0] > b[1])
    {
        b_pairs.push_back(vec2f(0, b[1]));
        b_pairs.push_back(vec2f(b[0], 2*M_PI));
    }
    else
        b_pairs.push_back(b);

    std::vector<vec2f> res;
    BOOST_FOREACH(const vec2f &ap, a_pairs)
    {
        BOOST_FOREACH(const vec2f &bp, b_pairs)
        {
            vec2f cand(interval_intersection(ap, bp));
            if(interval_okay(cand))
            {
                while(cand[0] >= 2*M_PI)
                    cand[0] -= 2*M_PI;
                while(cand[1] >= 2*M_PI)
                    cand[1] -= 2*M_PI;
                bool merge   = true;
                BOOST_FOREACH(vec2f &v, res)
                {
                    if(std::abs(v[1] - cand[0]) < 1e-5)
                    {
                        merge = false;
                        v[1]  = cand[1];
                        break;
                    }
                    else if(std::abs(v[0] - cand[1]) < 1e-5)
                    {
                        merge = false;
                        v[0]  = cand[0];
                        break;
                    }
                    else
                        merge = true;
                }
                if(merge)
                    res.push_back(cand);
            }
        }
    }

    return res;
}

struct interval
{
    interval() {}
    interval(const vec2f &r, bool i)
        : range(r), inside(i)
    {}

    vec2f range;
    bool  inside;
};

struct circle
{
    void plane(vec3f &n, float &d) const
    {
        n = vec3f(frame(0, 2),
                  frame(1, 2),
                  frame(2, 2));
        d = -tvmet::dot(n, center());
    }

    float arc(const vec3f &p) const
    {
        const vec3f dir(p);
        assert(std::abs(dir[2]) < 1e-6);
        const float theta(std::atan2(dir[1], dir[0]));
        return theta > 0 ? theta : 2*M_PI + theta;
    }

    vec3f point(const float theta) const
    {
        return vec3f(transform(frame, vec3f(radius*std::cos(theta), radius*std::sin(theta), 0.0)));
    }

    bool inside(const vec3f &p) const
    {
        const vec3f dir(p);
        assert(std::abs(dir[2]) < 1e-6);
        return length2(dir) < radius*radius;
    }

    vec3f center() const
    {
        return vec3f(frame(0, 3),
                     frame(1, 3),
                     frame(2, 3));
    }

    void center(const vec3f &c)
    {
        for(int i = 0; i < 3; ++i)
            frame(i, 3) = c[i];
    }

    mat4x4f inverse_frame() const
    {
        mat4x4f d0(frame);
        mat4x4f res(tvmet::identity<float, 4, 4>());

        rm_invert(d0.data(), res.data());
        return res;
    }

    mat4x4f frame;
    float   radius;
    vec2f   range;
};

aabb2d aabb_from_arc(const circle &c, const vec2f &r)
{
    vec3f       v(c.point(0)-c.center());
    const vec3f up(c.frame(0, 2), c.frame(1, 2), c.frame(2, 2));
    const float x_rot      = std::atan2(up[1], up[2]);
    v                      = transform(axis_angle_matrix(-x_rot, vec3f(1.0, 0.0, 0.0)), v);
    const float y_rot      = std::atan2(up[0], up[2]);
    v                      = transform(axis_angle_matrix(-y_rot, vec3f(0.0, 1.0, 0.0)), v);
    const float theta_bias = std::atan2(v[1], v[0]);

    aabb2d box;
    vec2f circle_range(r);
    if(circle_range[1] < circle_range[0])
        circle_range[1] += 2*M_PI;

    float theta = circle_range[0];
    const vec2f pt(sub<0,2>::vector(c.point(theta)));
    box.enclose_point(pt[0], pt[1]);
    theta = theta - std::fmod(theta, static_cast<float>(M_PI/2.0)) + M_PI/2 - theta_bias;
    while(theta < circle_range[1])
    {
        const vec2f pt(sub<0,2>::vector(c.point(theta)));
        box.enclose_point(pt[0], pt[1]);
        theta += M_PI/2;
    }
    const vec2f ptend(sub<0,2>::vector(c.point(circle_range[1])));
    box.enclose_point(ptend[0], ptend[1]);
    return box;
}

struct ray
{
    ray()
    {}

    ray(const vec3f &p0, const vec3f &p1)
        : origin(p0),
          dir(p1-p0),
          len(length(dir))
    {
        dir /= len;
    }

    vec3f point(float t) const
    {
        return vec3f(origin + t*dir);
    }

    // o + t * d = x
    // (x - c)^2 = r^2
    // (t*d + (o - c))^2 = r^2
    // d^2*t^2 + 2*d*(o-c)*t + (o-c)^2 = r^2
    // a = 1, b = 2*d*(o-c)  c = (o-c)^2 - r^2
    int intersect(vec2f &res, const circle &circ) const
    {
        const vec3f co(origin);//-circ.center());
        const float a   = 1;
        const float b   = 2*tvmet::dot(dir, co);
        const float c   = tvmet::dot(co, co) - circ.radius*circ.radius;
        const float det = b*b - 4*a*c;

        if(det < 0)
            return 0;

        if(det == 0)
        {
            res[0] = -b/(2*a);
            if(res[0] > 0.0 && res[0] < len)
                return 1;
            else
                return 0;
        }

        const float sterm       = std::sqrt(b*b - 4*a*c);
        const vec2f crossings(1.0/(2*a)*vec2f(-b-sterm, -b+sterm));
        int         crossing_no = 0;
        if(crossings[0] > 0.0 && crossings[0] < len)
        {
            res[crossing_no] = crossings[0];
            ++crossing_no;
        }
        if(crossings[1] > 0.0 && crossings[1] < len)
        {
            res[crossing_no] = crossings[1];
            ++crossing_no;
        }
        if(crossing_no == 2 && res[0] > res[1])
            std::swap(res[0], res[1]);

        return crossing_no;
    }

    vec3f origin;
    vec3f dir;
    float len;
};

//line is pt + t * n
//plane is x*n + d = 0
//intersection is (pt + t*n)*n + d = 0
// pt*n + t*n*n + d = 0
// t = -(d + pt*n)/n*n
static vec3f project_to_plane(const vec3f &pt, const vec3f &n, const float &d)
{
    const float t = -(d + tvmet::dot(pt, n))/length2(n);
    return vec3f(pt + t*n);
}

//plane is x*n + d = 0
// x is (pt[0), pt[1], ?)
// x = (d - (pt[0]*n[0] + pt[1]*n[1]))/n[2]
static vec3f z_project_to_plane(const vec3f &pt, const vec3f &n, const float &d)
{
    return vec3f(pt[0], pt[1], (-d - tvmet::dot(sub<0,2>::vector(n), sub<0,2>::vector(pt)))/n[2]);
}

// assuming clockwise ordering
struct simple_polygon : std::vector<vec3f>
{
    ray line(int e) const
    {
        return ray((*this)[e], (*this)[(e+1)%size()]);
    }

    void project_to_plane(const vec3f &n, const float d)
    {
        BOOST_FOREACH(vec3f &pt, (*this))
        {
            pt = ::project_to_plane(pt, n, d);
        }
    }

    void z_project_to_plane(const vec3f &n, const float d)
    {
        BOOST_FOREACH(vec3f &pt, (*this))
        {
            pt = ::z_project_to_plane(pt, n, d);
        }
    }

    void transform(const mat4x4f &m)
    {
        BOOST_FOREACH(vec3f &pt, (*this))
        {
            pt = ::transform(m, pt);
        }
    }

    bool contained_by_circle(const circle &c) const
    {
        BOOST_FOREACH(const vec3f &v, (*this))
        {
            if(!c.inside(v))
                return false;
        }
        return true;
    }

    // x = my + b
    // x = dx/dy * y + b
    // x_0 = dx/dy * y0 + b
    // b = (x0-dx/dy*y0)
    bool point_in_polygon(const vec2f &p) const
    {
        bool inside = false;
        for(int i = 0; i < static_cast<int>(size()); ++i)
        {
            const vec2f &l0 = sub<0,2>::vector((*this)[i]);
            const vec2f &l1 = sub<0,2>::vector((*this)[(i+1)%static_cast<int>(size())]);
            if((l0[1] - p[1])*(l1[1] - p[1]) < 0)
            {
                const float m = (l0[0]-l1[0])/(l0[1]-l1[1]);
                const float b = l0[0] - m*l0[1];
                const float x = m*p[1] + b;
                if(!(std::abs(l1[0] - (m*l1[1] + b)) < 1e-5))
                {
                    std::cout << l1[0] - (m*l1[1] + b) << std::endl;
                    assert(0);
                }
                if(x >= p[0])
                    continue;
                inside = !inside;
            }
        }
        return inside;
    }
};

std::vector<interval> chop_circle(const simple_polygon &p, const circle &c)
{
    circle_partition res;
    bool             looking_for_single = false;
    float            last_single;
    for(int i = 0; i < static_cast<int>(p.size()); ++i)
    {
        const bool p_inside(c.inside(p[i]));
        const ray  r(p.line(i));
        vec2f      crossings;
        const int  crossing_no(r.intersect(crossings, c));
        if(crossing_no == 2)
        {
            vec2f param(c.arc(r.point(crossings[0])),
                        c.arc(r.point(crossings[1])));
            if(param[0] > param[0])
                std::swap(param[0], param[1]);
            res.insert(param[0], p_inside);
            res.insert(param[1], !p_inside);
        }
        else if(crossing_no == 1)
        {
            float this_param(c.arc(r.point(crossings[0])));
            if(looking_for_single)
            {
                res.insert(last_single, !p_inside);
                res.insert(this_param, p_inside);
            }
            else
            {
                last_single = this_param;
                looking_for_single = true;
            }
        }
    }

    std::vector<interval> ires;
    if(res.divisions.empty())
    {
        const bool center_in_poly(p.point_in_polygon(sub<0,2>::vector(c.center())));
        const bool poly_in_circle(p.contained_by_circle(c));
        ires.push_back(interval(c.range, center_in_poly && !poly_in_circle));
        return ires;
    }

    std::map<float, bool>::iterator current = res.divisions.begin();
    for(; current != res.divisions.end(); ++current)
    {
        std::map<float, bool>::iterator next(current);
        res.advance(next);
        vec2f cand_range(current->first, next->first);
        BOOST_FOREACH(const vec2f &v, interval_overlap(cand_range, c.range))
        {
            ires.push_back(interval(v, current->second));
        }
    }
    return ires;
}

static void cairo_draw_arc(cairo_t *cr, const circle &c, const vec2f &range, bool inside, const cairo_matrix_t *cmat)
{
    cairo_set_matrix(cr, cmat);
    if(inside)
        cairo_set_source_rgba(cr, 1.0, 0.0, 0.0, 1.0);
    else
        cairo_set_source_rgba(cr, 0.0, 0.0, 1.0, 1.0);
    cairo_set_matrix(cr, cmat);
    cairo_arc(cr, c.center()[0], c.center()[1], c.radius,
              range[0], range[1]);
    cairo_identity_matrix(cr);
    cairo_set_line_width(cr, 2.0);
    cairo_stroke(cr);
}

static void gl_draw_arc(const circle &c, const vec2f &range, bool inside)
{
    if(inside)
        glColor4f(1.0, 0.0, 0.0, 1.0);
    else
        glColor4f(0.0, 0.0, 1.0, 1.0);
    glBegin(GL_LINE_STRIP);
    const int N = 100;
    vec2f draw_range(range);
    if(draw_range[1] < draw_range[0])
        draw_range[1] += 2*M_PI;
    for(int i = 0; i < N; ++i)
    {
        const float theta = draw_range[0] + (draw_range[1]-draw_range[0])*i/static_cast<float>(N-1);
        glVertex3fv(c.point(theta).data());
    }
    glEnd();

    const aabb2d box(aabb_from_arc(c, range));
    glColor3f(0.4, 0.8, 0.4);
    glBegin(GL_QUADS);
    glVertex2f(box.bounds[0][0], box.bounds[0][1]);
    glVertex2f(box.bounds[1][0], box.bounds[0][1]);
    glVertex2f(box.bounds[1][0], box.bounds[1][1]);
    glVertex2f(box.bounds[0][0], box.bounds[1][1]);
    glEnd();
}

class fltkview : public Fl_Gl_Window
{
public:
    fltkview(int x, int y, int w, int h, const char *l) : Fl_Gl_Window(x, y, w, h, l),
                                                          lastpick(0),
                                                          glew_state(GLEW_OK+1),
                                                          sp(0),
                                                          c(0)
    {
        this->resizable(this);
    }

    void init_glew()
    {
        glew_state = glewInit();
        if (GLEW_OK != glew_state)
        {
            /* Problem: glewInit failed, something is seriously wrong. */
            std::cerr << "Error: " << glewGetErrorString(glew_state)  << std::endl;
        }
        std::cerr << "Status: Using GLEW " << glewGetString(GLEW_VERSION) << std::endl;
    }

    void draw()
    {
        if (!valid())
        {
            glViewport(0, 0, w(), h());
            glClearColor(0.0, 0.0, 0.0, 0.0);

            if(GLEW_OK != glew_state)
                init_glew();

            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        }

        glClear(GL_COLOR_BUFFER_BIT);

        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();

        vec2f lo, hi;
        cscale_to_box(lo, hi, center, scale, vec2i(w(), h()));
        glOrtho(lo[0], hi[0], lo[1], hi[1], -100, 100);

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

        if(c)
        {
            vec3f n;
            float d;
            c->plane(n, d);

            simple_polygon sp_copy(*sp);
            sp_copy.z_project_to_plane(n, d);
            sp_copy.transform(c->inverse_frame());

            std::vector<interval> cpi(chop_circle(sp_copy, *c));

            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
            BOOST_FOREACH(const interval &i, cpi)
            {
                gl_draw_arc(*c, i.range, i.inside);
            }
        }

        if(sp)
        {
            assert(c);
            vec3f n;
            float d;
            c->plane(n, d);

            simple_polygon sp_copy(*sp);
            sp_copy.z_project_to_plane(n, d);
            // sp_copy.transform(c->inverse_frame());

            glColor3f(1.0, 1.0, 1.0);
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
            glBegin(GL_LINE_LOOP);
            BOOST_FOREACH(const vec3f &p, sp_copy)
            {
                glVertex3fv(p.data());
            }
            glEnd();
        }

        glFlush();
        glFinish();
    }

    int handle(int event)
    {
        switch(event)
        {
        case FL_PUSH:
            {
                const vec2i xy(Fl::event_x(),
                               Fl::event_y());
                const vec2f world(world_point(vec2i(xy[0], h()-xy[1]), center, scale, vec2i(w(), h())));

                lastpick = world;
            }
            take_focus();
            return 1;
        case FL_RELEASE:
            {
            }
            return 1;
        case FL_DRAG:
            {
                const vec2i xy(Fl::event_x(),
                               Fl::event_y());
                const vec2f world(world_point(vec2i(xy[0], h()-xy[1]), center, scale, vec2i(w(), h())));
                vec2f dvec(0);
                if(Fl::event_button() == FL_MIDDLE_MOUSE)
                {
                    dvec = vec2f(world - lastpick);
                    center -= dvec;
                    redraw();
                    lastpick = world-dvec;
                }
                else if(Fl::event_button() == FL_LEFT_MOUSE)
                {
                    dvec = vec2f(world - lastpick);
                    if(c)
                    {
                        vec3f cent(c->center());
                        sub<0,2>::vector(cent) += dvec;
                        c->center(cent);
                    }
                    redraw();
                    lastpick = world;
                }
            }
            take_focus();
            return 1;
        case FL_KEYBOARD:
            switch(Fl::event_key())
            {
            case ' ':
                redraw();
                break;
            }
            return 1;
        case FL_MOUSEWHEEL:
            {
                const vec2i xy(Fl::event_x(),
                               Fl::event_y());
                const vec2i dxy(Fl::event_dx(),
                                Fl::event_dy());
                const float fy = copysign(0.5, dxy[1]);

                if(Fl::event_state() & FL_SHIFT)
                {
                    if(c)
                        c->radius *= std::pow(2.0, fy);
                }
                else
                    scale *= std::pow(2.0, fy);

                redraw();
            }
            take_focus();
            return 1;
        default:
            // pass other events to the base class...
            return Fl_Gl_Window::handle(event);
        }
    }

    vec2f  lastpick;
    vec2f  center;
    float  scale;
    GLuint glew_state;

    simple_polygon        *sp;
    circle                *c;
};

int main(int argc, char *argv[])
{
    simple_polygon sp;
    sp.push_back(vec3f(-2.0,  2.0, 0.0));
    sp.push_back(vec3f(-2.0, -2.0, 0.0));
    sp.push_back(vec3f(-1.0, -2.0, 0.0));
    sp.push_back(vec3f( 0.0,  0.0, 0.0));
    sp.push_back(vec3f( 1.0, -2.0, 0.0));
    sp.push_back(vec3f( 2.0, -2.0, 0.0));
    sp.push_back(vec3f( 2.0,  2.0, 0.0));

    circle c;
    c.frame = axis_angle_matrix(M_PI/4, vec3f(0, 1, 0))*axis_angle_matrix(M_PI/4, vec3f(0, 0, 1));
    c.center(vec3f(0));
    c.radius = 2.0;

    c.range  = vec2f(M_PI, M_PI/2);

    fltkview mv(0, 0, 500, 500, "fltk View");

    box_to_cscale(mv.center, mv.scale,
                  vec2f(-10, -10), vec2f(10, 10),
                  vec2i(500, 500));

    mv.sp = &sp;
    mv.c  = &c;
    mv.take_focus();
    Fl::visual(FL_DOUBLE|FL_DEPTH);

    mv.show(1, argv);
    return Fl::run();
}

