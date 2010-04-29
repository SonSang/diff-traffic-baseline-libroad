#include <GL/glew.h>
#include <FL/Fl.H>
#include <FL/Fl_Gl_Window.H>
#include <FL/Fl_Menu_Button.H>
#include <FL/gl.h>
#include <FL/glu.h>
#include <FL/glut.h>
#include "libroad/hwm_network.hpp"
#include "geometric.hpp"

static void box_to_cscale(vec2f &center, float &scale,
                          const vec2f &low, const vec2f &high,
                          const vec2i &window)
{
    center = vec2f(low+high)/2;

    const vec2f dv(high-low);
    const float view_aspect  = static_cast<float>(window[0])/window[1];
    const float scene_aspect = dv[0]/dv[1];

    if(view_aspect > scene_aspect)
        scale = dv[0];
    else
        scale = dv[1];
}

static void cscale_to_box(vec2f &low, vec2f &high,
                          const vec2f &center, const float &scale,
                          const vec2i &window)
{
    vec2f dv;
    if(window[0] > window[1])
    {
        dv[0] =  scale*window[0]/window[1];
        dv[1] =  scale;
    }
    else
    {
        dv[0] =  scale;
        dv[1] =  scale*window[1]/window[0];
    }
    low  = center - dv/2;
    high = center + dv/2;
}

class fltkview : public Fl_Gl_Window
{
public:
    fltkview(int x, int y, int w, int h, const char *l) : Fl_Gl_Window(x, y, w, h, l),
                                                          lastmouse(0),
                                                          lastpick(0),
                                                          net(0),
                                                          netaux(0),
                                                          glew_state(GLEW_OK+1),
                                                          tex_(0)
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
        if(!glIsTexture(tex_))
        {
            glGenTextures(1, &tex_);
            glBindTexture (GL_TEXTURE_2D, tex_);
            glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);

            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
            glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
            retex(center, scale, vec2i(w(), h()));
        }
    }

    void retex(const vec2f &my_center, const float my_scale, const vec2i &im_res)
    {
        cairo_surface_t *cs = cairo_image_surface_create(CAIRO_FORMAT_ARGB32,
                                                         im_res[0],
                                                         im_res[1]);
        cairo_t         *cr = cairo_create(cs);
        cairo_set_source_rgba(cr, 0.5, 0, 0, 1.0);
        cairo_set_operator(cr, CAIRO_OPERATOR_SOURCE);
        cairo_paint(cr);

        cairo_set_operator(cr, CAIRO_OPERATOR_OVER);

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

        cairo_set_line_width(cr, 0.5);
        netaux->cairo_roads(cr);

        cairo_destroy(cr);

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

        cscale_to_box(tex_low, tex_high, my_center, my_scale, im_res);
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
        }

        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();

        vec2f lo, hi;
        cscale_to_box(lo, hi, center, scale, vec2i(w(), h()));
        gluOrtho2D(lo[0], hi[0], lo[1], hi[1]);

        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();

        glEnable(GL_TEXTURE_2D);
        glBindTexture (GL_TEXTURE_2D, tex_);

        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        glPushMatrix();

        glBegin(GL_QUADS);
        glTexCoord2f(0.0, 0.0);
        glVertex2fv(tex_low.data());
        glTexCoord2f(1.0, 0.0);
        glVertex2f(tex_high[0], tex_low[1]);
        glTexCoord2f(1.0, 1.0);
        glVertex2fv(tex_high.data());
        glTexCoord2f(0.0, 1.0);
        glVertex2f(tex_low[0], tex_high[1]);
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
                const vec2f fxy( static_cast<float>(xy[0])/(w()-1),
                                 1-static_cast<float>(xy[1])/(h()-1));
                if(Fl::event_button() == FL_LEFT_MOUSE)
                    lastpick  = fxy;
                lastmouse = fxy;
            }
            take_focus();
            return 1;
        case FL_DRAG:
            {
                const vec2i xy(Fl::event_x(),
                               Fl::event_y());
                const vec2f fxy( static_cast<float>(xy[0])/(w()-1),
                                 1-static_cast<float>(xy[1])/(h()-1));
                if(Fl::event_button() == FL_LEFT_MOUSE)
                {
                    vec2f fac;
                    if(w() > h())
                        fac = vec2f(static_cast<float>(w())/h(), 1.0);
                    else
                        fac = vec2f(1.0, static_cast<float>(h())/w());

                    const vec2f dvec((fxy - lastpick)*fac*scale);
                    center   -= dvec;
                    lastpick  = fxy;
                    redraw();
                }
                lastmouse = fxy;
            }
            take_focus();
            return 1;
        case FL_KEYBOARD:
            switch(Fl::event_key())
            {
            case ' ':
                retex(center, scale, vec2i(w(), h()));
                redraw();
                break;
            }
            return 1;
        case FL_MOUSEWHEEL:
            {
                const int   x   = Fl::event_dx();
                const int   y   = Fl::event_dy();
                const float fy  = copysign(0.5, y);
                scale          *= std::pow(2.0, fy);
                redraw();
            }
            take_focus();
            return 1;
        default:
            // pass other events to the base class...
            return Fl_Gl_Window::handle(event);
        }
    }

    vec2f lastmouse;
    vec2f lastpick;

    hwm::network     *net;
    hwm::network_aux *netaux;
    vec2f             tex_low;
    vec2f             tex_high;
    vec2f             center;
    float             scale;

    GLuint glew_state;
    GLuint tex_;
};

int main(int argc, char *argv[])
{
    std::cout << libroad_package_string() << std::endl;
    if(argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " <input network>" << std::endl;
        return 1;
    }
    hwm::network net(hwm::load_xml_network(argv[1], vec3f(1.0, 1.0, 1.0f)));

    net.build_intersections();
    net.build_fictitious_lanes();
    net.auto_scale_memberships();
    net.center();
    std::cerr << "HWM net loaded successfully" << std::endl;

    try
    {
        net.check();
        std::cerr << "HWM net checks out" << std::endl;
    }
    catch(std::runtime_error &e)
    {
        std::cerr << "HWM net doesn't check out: " << e.what() << std::endl;
        exit(1);
    }

    hwm::network_aux neta(net);

    fltkview mv(0, 0, 500, 500, "fltk View");
    mv.net    = &net;
    mv.netaux = &neta;

    vec3f low(FLT_MAX);
    vec3f high(-FLT_MAX);
    net.bounding_box(low, high);
    std::cout << low << " " << high << std::endl;
    box_to_cscale(mv.center, mv.scale, sub<0,2>::vector(low), sub<0,2>::vector(high), vec2i(500,500));

    mv.take_focus();
    Fl::visual(FL_DOUBLE|FL_DEPTH);

    mv.show(1, argv);
    return Fl::run();
}
