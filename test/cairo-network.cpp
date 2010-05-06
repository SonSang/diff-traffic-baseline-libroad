#include <Magick++.h>
#include <GL/glew.h>
#include <FL/Fl.H>
#include <FL/Fl_Gl_Window.H>
#include <FL/Fl_Menu_Button.H>
#include <FL/gl.h>
#include <FL/glu.h>
#include <FL/glut.h>
#include "libroad/hwm_network.hpp"
#include "libroad/geometric.hpp"
#include "visual_geometric.hpp"

class fltkview : public Fl_Gl_Window
{
public:
    fltkview(int x, int y, int w, int h, const char *l) : Fl_Gl_Window(x, y, w, h, l),
                                                          lastpick(0),
                                                          net(0),
                                                          netaux(0),
                                                          glew_state(GLEW_OK+1),
                                                          road_tex_(0),
                                                          background_tex_(0),
                                                          back_image(0),
                                                          back_image_center(0),
                                                          back_image_scale(1),
                                                          back_image_yscale(1),
                                                          overlay_tex_(0),
                                                          drawing(false)
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
        if(!glIsTexture(road_tex_))
        {
            glGenTextures(1, &road_tex_);
            glBindTexture (GL_TEXTURE_2D, road_tex_);
            glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);

            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
            glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
            retex_roads(center, scale, vec2i(w(), h()));
        }
        if(back_image && !glIsTexture(background_tex_))
        {
            glGenTextures(1, &background_tex_);
            glBindTexture (GL_TEXTURE_2D, background_tex_);
            glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);

            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
            glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);

            Magick::Image im(*back_image);
            im.flip();
            back_image_dim = vec2i(im.columns(), im.rows());
            unsigned char *pix = new unsigned char[back_image_dim[0]*back_image_dim[1]*4];
            im.write(0, 0, back_image_dim[0], back_image_dim[1], "RGBA", Magick::CharPixel, pix);
            glTexImage2D (GL_TEXTURE_2D,
                          0,
                          GL_RGBA,
                          back_image_dim[0],
                          back_image_dim[1],
                          0,
                          GL_RGBA,
                          GL_UNSIGNED_BYTE,
                          pix);
            delete[] pix;
        }
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

    void retex_roads(const vec2f &my_center, const float my_scale, const vec2i &im_res)
    {
        cairo_surface_t *cs = cairo_image_surface_create(CAIRO_FORMAT_ARGB32,
                                                         im_res[0],
                                                         im_res[1]);
        cairo_t         *cr = cairo_create(cs);
        cairo_set_source_rgba(cr, 0.0, 0, 0, 0.0);
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

        glBindTexture (GL_TEXTURE_2D, road_tex_);
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
        cairo_set_operator(cr, CAIRO_OPERATOR_SOURCE);

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

        BOOST_FOREACH(const aabb2d &r, rectangles)
        {
            cairo_set_matrix(cr, &cmat);
            cairo_rectangle(cr, r.bounds[0][0], r.bounds[0][1], r.bounds[1][0]-r.bounds[0][0], r.bounds[1][1]-r.bounds[0][1]);
            cairo_set_source_rgba(cr, 67/255.0, 127/255.0, 195/255.0, 0.5);
            cairo_fill_preserve(cr);
            cairo_set_source_rgba(cr, 17/255.0, 129/255.0, 255/255.0, 0.7);
            cairo_identity_matrix(cr);
            cairo_set_line_width(cr, 2.0);
            cairo_stroke(cr);
        }

        if(drawing)
        {
            const vec2f low(std::min(first_point[0], second_point[0]),
                            std::min(first_point[1], second_point[1]));
            const vec2f high(std::max(first_point[0], second_point[0]),
                             std::max(first_point[1], second_point[1]));

            cairo_set_matrix(cr, &cmat);
            cairo_rectangle(cr, low[0], low[1], high[0]-low[0], high[1]-low[1]);
            cairo_set_source_rgba(cr, 67/255.0, 127/255.0, 195/255.0, 0.5);
            cairo_fill_preserve(cr);
            cairo_set_source_rgba(cr, 17/255.0, 129/255.0, 255/255.0, 0.7);
            cairo_identity_matrix(cr);
            cairo_set_line_width(cr, 2.0);
            cairo_stroke(cr);
        }

        BOOST_FOREACH(const hwm::road_spatial::entry &e, query_results)
        {
            cairo_set_matrix(cr, &cmat);
            const aabb2d rect(e.r->rep.bound_feature2d(e.feature));
            cairo_rectangle(cr, rect.bounds[0][0], rect.bounds[0][1], rect.bounds[1][0]-rect.bounds[0][0], rect.bounds[1][1]-rect.bounds[0][1]);
            cairo_set_source_rgba(cr, 195/255.0, 127/255.0, 67/255.0, 0.5);
            cairo_fill_preserve(cr);
            cairo_set_source_rgba(cr, 255/255.0, 129/255.0, 217/255.0, 0.7);
            cairo_identity_matrix(cr);
            cairo_set_line_width(cr, 2.0);
            cairo_stroke(cr);
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
        if(back_image)
        {
            glBindTexture (GL_TEXTURE_2D, background_tex_);

            glPushMatrix();
            glTranslatef(-back_image_center[0], -back_image_center[1], 0);
            glScalef    (back_image_scale, back_image_yscale*back_image_scale,   1);

            glTranslatef(-back_image_dim[0]/2, -back_image_dim[1]/2, 0);
            glBegin(GL_QUADS);
            glTexCoord2f(0.0, 0.0);
            glVertex2i(0, 0);
            glTexCoord2f(1.0, 0.0);
            glVertex2i(back_image_dim[0], 0);
            glTexCoord2f(1.0, 1.0);
            glVertex2iv(back_image_dim.data());
            glTexCoord2f(0.0, 1.0);
            glVertex2i(0, back_image_dim[1]);
            glEnd();
            glPopMatrix();
        }

        glBindTexture (GL_TEXTURE_2D, road_tex_);

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

                if(Fl::event_button() == FL_LEFT_MOUSE)
                    first_point = world;

                lastpick = world;
            }
            take_focus();
            return 1;
        case FL_RELEASE:
            {
                if(Fl::event_button() == FL_LEFT_MOUSE)
                {
                    if(drawing)
                    {
                        rectangles.clear();
                        rectangles.push_back(aabb2d());
                        rectangles.back().enclose_point(first_point[0], first_point[1]);
                        rectangles.back().enclose_point(second_point[0], second_point[1]);
                        drawing = false;
                        query_results = netaux->road_space.query(rectangles.back());
                        redraw();
                    }
                }
            }
            return 1;
        case FL_DRAG:
            {
                const vec2i xy(Fl::event_x(),
                               Fl::event_y());
                const vec2f world(world_point(vec2i(xy[0], h()-xy[1]), center, scale, vec2i(w(), h())));
                vec2f dvec(0);
                if(Fl::event_button() == FL_LEFT_MOUSE)
                {
                    drawing      = true;
                    second_point = world;
                    redraw();
                }
                else if(Fl::event_button() == FL_MIDDLE_MOUSE)
                {
                    dvec = vec2f(world - lastpick);
                    center -= dvec;
                    redraw();
                }
                else if(Fl::event_button() == FL_RIGHT_MOUSE)
                {
                    dvec = vec2f(world - lastpick);
                    back_image_center -= dvec;
                    redraw();
                }
                lastpick = world-dvec;
            }
            take_focus();
            return 1;
        case FL_KEYBOARD:
            switch(Fl::event_key())
            {
            case ' ':
                retex_roads(center, scale, vec2i(w(), h()));
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
                    if(Fl::event_state() & FL_CTRL)
                        back_image_yscale *= std::pow(2.0, 0.1*fy);
                    else
                        back_image_scale  *= std::pow(2.0, 0.5*fy);
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

    vec2f lastpick;

    hwm::network     *net;
    hwm::network_aux *netaux;
    vec2f             tex_low;
    vec2f             tex_high;
    vec2f             center;
    float             scale;

    GLuint   glew_state;
    GLuint   road_tex_;
    GLuint   background_tex_;
    char   **back_image;
    vec2i    back_image_dim;
    vec2f    back_image_center;
    float    back_image_scale;
    float    back_image_yscale;

    GLuint                 overlay_tex_;

    std::vector<aabb2d>                   rectangles;
    bool                                  drawing;
    vec2f                                 first_point;
    vec2f                                 second_point;
    std::vector<hwm::road_spatial::entry> query_results;
};

int main(int argc, char *argv[])
{
    std::cout << libroad_package_string() << std::endl;
    if(argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " <input network> [background image]" << std::endl;
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
    mv.net            = &net;
    mv.netaux         = &neta;
    if(argc == 3)
        mv.back_image = argv+2;

    vec3f low(FLT_MAX);
    vec3f high(-FLT_MAX);
    net.bounding_box(low, high);
    box_to_cscale(mv.center, mv.scale, sub<0,2>::vector(low), sub<0,2>::vector(high), vec2i(500,500));

    mv.take_focus();
    Fl::visual(FL_DOUBLE|FL_DEPTH);

    mv.show(1, argv);
    return Fl::run();
}
