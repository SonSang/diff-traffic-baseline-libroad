#include "libroad/libroad_common.hpp"
#include <GL/glew.h>
#include <FL/Fl.H>
#include <FL/Fl_Gl_Window.H>
#include <FL/Fl_Menu_Button.H>
#include <FL/gl.h>
#include <FL/glu.h>
#include <FL/glut.h>
#include "libroad/hwm_network.hpp"
#include "../../test/geometric.hpp"

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

struct circle
{
    float arc(const vec3f &p) const
    {
        const vec3f dir(p - center);
        const float theta(std::atan2(dir[1], dir[0]));
        return theta > 0 ? theta : 2*M_PI + theta;
    }

    bool inside(const vec3f &p) const
    {
        return length2(p-center) < radius*radius;
    }

    float radius;
    vec3f center;
    vec2f range;
};

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
        const vec3f co(origin-circ.center);
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

// assuming clockwise ordering
struct simple_polygon : std::vector<vec3f>
{
    ray line(int e) const
    {
        return ray((*this)[e], (*this)[(e+1)%size()]);
    }

    bool contains_circle(const circle &c) const
    {
        const bool first_inside(c.inside((*this)[0]));
        for(size_t i = 1; i < size(); ++i)
        {
            if(c.inside((*this)[i]) != first_inside)
                return true;
        }
        return false;
    }
};

circle_partition chop_circle(const simple_polygon &p, const circle &c)
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

    return res;
}

static void draw_arc(cairo_t *cr, const circle &c, const vec2f &range, bool inside, const cairo_matrix_t *cmat)
{
    cairo_set_matrix(cr, cmat);
    if(inside)
        cairo_set_source_rgba(cr, 1.0, 0.0, 0.0, 1.0);
    else
        cairo_set_source_rgba(cr, 0.0, 0.0, 1.0, 1.0);
    cairo_set_matrix(cr, cmat);
    cairo_arc(cr, c.center[0], c.center[1], c.radius,
              range[0], range[1]);
    cairo_identity_matrix(cr);
    cairo_set_line_width(cr, 2.0);
    cairo_stroke(cr);
}

class fltkview : public Fl_Gl_Window
{
public:
    fltkview(int x, int y, int w, int h, const char *l) : Fl_Gl_Window(x, y, w, h, l),
                                                          lastpick(0),
                                                          glew_state(GLEW_OK+1),
                                                          overlay_tex_(0),
                                                          sp(0),
                                                          c(0),
                                                          cp(0)
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

    void init_textures()
    {
        if(!glIsTexture(overlay_tex_))
        {
            glGenTextures(1, &overlay_tex_);
            glBindTexture (GL_TEXTURE_2D, overlay_tex_);
            glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);

            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
            glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
            retex_overlay(center, scale, vec2i(w(), h()));
        }
    }

    void retex_overlay(const vec2f &my_center, const float my_scale, const vec2i &im_res)
    {
        cairo_surface_t *cs = cairo_image_surface_create(CAIRO_FORMAT_ARGB32,
                                                         im_res[0],
                                                         im_res[1]);
        cairo_t         *cr = cairo_create(cs);
        cairo_set_source_rgba(cr, 0.0, 0, 0, 0.0);
        cairo_set_operator(cr, CAIRO_OPERATOR_SOURCE);
        cairo_paint(cr);

        cairo_set_line_cap(cr, CAIRO_LINE_CAP_ROUND);

        cairo_translate(cr,
                        im_res[0]/2,
                        im_res[1]/2);

        cairo_scale(cr,
                    im_res[0]/my_scale,
                    im_res[1]/my_scale);


        if(im_res[0] > im_res[1])
            cairo_scale(cr,
                        static_cast<float>(im_res[1])/im_res[0],
                        1.0);
        else
            cairo_scale(cr,
                        1.0,
                        static_cast<float>(im_res[0])/im_res[1]);

        cairo_translate(cr,
                        -my_center[0],
                        -my_center[1]);

        cairo_matrix_t cmat;
        cairo_get_matrix(cr, &cmat);

        cairo_set_operator(cr, CAIRO_OPERATOR_OVER);
        if(sp)
        {
            cairo_set_matrix(cr, &cmat);
            cairo_set_source_rgba(cr, 1.0, 1.0, 1.0, 1.0);
            cairo_move_to(cr, sp->front()[0], sp->front()[1]);
            for(int i = 1; i < static_cast<int>(sp->size()); ++i)
                cairo_line_to(cr, (*sp)[i][0], (*sp)[i][1]);
            cairo_line_to(cr, (*sp)[0][0], (*sp)[0][1]);
            cairo_identity_matrix(cr);
            cairo_set_line_width(cr, 2.0);
            cairo_stroke(cr);
        }

        if(cp)
        {
            cairo_identity_matrix(cr);
            cairo_set_line_width(cr, 2.0);
            cairo_set_matrix(cr, &cmat);

            assert(c);
            if(cp->divisions.empty())
                draw_arc(cr, *c, c->range, sp->contains_circle(*c), &cmat);
            else
            {
                std::map<float, bool>::iterator start(cp->divisions.upper_bound(c->range[0]));
                std::map<float, bool>::iterator end  (cp->divisions.upper_bound(c->range[1]));
                if(start == end)
                {
                    cp->back_up(start);
                    draw_arc(cr, *c, c->range, start->second, &cmat);
                }
                else
                {
                    cp->back_up(start);
                    cp->back_up(end);
                    {
                        std::map<float, bool>::iterator next(start);
                        cp->advance(next);
                        draw_arc(cr, *c, vec2f(c->range[0], next->first), start->second, &cmat);
                    }
                    std::map<float, bool>::iterator current = start;
                    cp->advance(current);
                    for(; current != end; cp->advance(current))
                    {
                        std::map<float, bool>::iterator next(current);
                        cp->advance(next);
                        draw_arc(cr, *c, vec2f(current->first, next->first), current->second, &cmat);
                    }
                    draw_arc(cr, *c, vec2f(end->first, c->range[1]), end->second, &cmat);
                }
            }
        }

        cairo_destroy(cr);

        glBindTexture(GL_TEXTURE_2D, overlay_tex_);
        glTexImage2D (GL_TEXTURE_2D,
                      0,
                      GL_RGBA,
                      im_res[0],
                      im_res[1],
                      0,
                      GL_BGRA,
                      GL_UNSIGNED_BYTE,
                      cairo_image_surface_get_data(cs));

        cairo_surface_destroy(cs);
    }

    void draw()
    {
        if (!valid())
        {
            glViewport(0, 0, w(), h());
            glClearColor(0.0, 0.0, 0.0, 0.0);

            if(GLEW_OK != glew_state)
                init_glew();

            init_textures();
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        }

        glClear(GL_COLOR_BUFFER_BIT);

        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();

        vec2f lo, hi;
        cscale_to_box(lo, hi, center, scale, vec2i(w(), h()));
        gluOrtho2D(lo[0], hi[0], lo[1], hi[1]);

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

        glEnable(GL_TEXTURE_2D);

        glBindTexture (GL_TEXTURE_2D, overlay_tex_);
        retex_overlay(center, scale, vec2i(w(), h()));
        glPushMatrix();
        glBegin(GL_QUADS);
        glTexCoord2f(0.0, 0.0);
        glVertex2fv(lo.data());
        glTexCoord2f(1.0, 0.0);
        glVertex2f(hi[0], lo[1]);
        glTexCoord2f(1.0, 1.0);
        glVertex2fv(hi.data());
        glTexCoord2f(0.0, 1.0);
        glVertex2f(lo[0], hi[1]);
        glEnd();
        glPopMatrix();

        glDisable(GL_TEXTURE_2D);

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
                        sub<0,2>::vector(c->center) += dvec;
                        std::cout << c->center << std::endl;
                        if(cp)
                            *cp = circle_partition(chop_circle(*sp, *c));
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
                    {
                        c->radius *= std::pow(2.0, fy);
                        if(cp)
                            *cp = circle_partition(chop_circle(*sp, *c));
                    }
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
    GLuint overlay_tex_;

    simple_polygon   *sp;
    circle           *c;
    circle_partition *cp;
};

int main(int argc, char *argv[])
{
    simple_polygon sp;
    sp.push_back(vec3f(-2.0,  2.0, 0.0));
    sp.push_back(vec3f(-2.0, -2.0, 0.0));
    sp.push_back(vec3f(-1.0, -2.0, 0.0));
    sp.push_back(vec3f( 0.0, -4.0, 0.0));
    sp.push_back(vec3f( 1.0, -2.0, 0.0));
    sp.push_back(vec3f( 2.0, -2.0, 0.0));
    sp.push_back(vec3f( 2.0,  2.0, 0.0));

    circle c;
    c.center = vec3f(3.78271, -0.27487, 0);
    c.radius = 6.0;
    c.range  = vec2f(3*M_PI/2, M_PI/2);

    fltkview mv(0, 0, 500, 500, "fltk View");

    box_to_cscale(mv.center, mv.scale,
                  vec2f(-10, -10), vec2f(10, 10),
                  vec2i(500, 500));

    mv.sp = &sp;
    mv.c  = &c;
    mv.cp = new circle_partition(chop_circle(sp, c));
    mv.take_focus();
    Fl::visual(FL_DOUBLE|FL_DEPTH);

    mv.show(1, argv);
    return Fl::run();
}

