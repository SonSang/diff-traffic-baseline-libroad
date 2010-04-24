#include <Magick++.h>
#include <GL/glew.h>
#include <FL/Fl.H>
#include <FL/Fl_Gl_Window.H>
#include <FL/Fl_Menu_Button.H>
#include <FL/gl.h>
#include <FL/glu.h>
#include "FL/glut.h"
#include "arcball.hpp"
#include "im_heightfield.hpp"

#include "libroad/osm_network.hpp"
#include "libroad/hwm_network.hpp"

class fltkview : public Fl_Gl_Window
{
public:
    fltkview(int x, int y, int w, int h, const char *l) : Fl_Gl_Window(x, y, w, h, l),
                                                          zoom(2.0),
                                                          glew_state(GLEW_OK+1),
                                                          tex_(0)
    {
        lastmouse[0] = 0.0f;
        lastmouse[1] = 0.0f;

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

    // void init_textures()
    // {
    //     if(!glIsTexture(tex_))
    //     {
    //         glGenTextures(1, &tex_);
    //         glEnable(GL_TEXTURE_2D);
    //         glBindTexture (GL_TEXTURE_2D, tex_);
    //         glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
    //         glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    //         glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    //         glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    //         glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
    //         glDisable(GL_TEXTURE_2D);
    //     }
    // }

    void draw()
    {
        if (!valid())
        {
            glViewport(0, 0, w(), h());
            glMatrixMode(GL_PROJECTION);
            glLoadIdentity();
            gluPerspective(45.0f, (GLfloat)w()/(GLfloat)h(), 100.0f, 50000.0f);

            glMatrixMode(GL_MODELVIEW);
            glClearColor(0.0, 0.0, 0.0, 0.0);

            glEnable(GL_DEPTH_TEST);

            glHint(GL_LINE_SMOOTH_HINT,    GL_NICEST);
            glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);

            glEnable(GL_LINE_SMOOTH);

            if(GLEW_OK != glew_state)
                init_glew();

            // init_textures();
        }

        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

        glLoadIdentity();

        glTranslatef(0.0f, 0.0f, -std::pow(2.0f, zoom));

        nav.get_rotation();
        glMultMatrixd(nav.world_mat());

        // glEnable(GL_TEXTURE_2D);
        // glBindTexture (GL_TEXTURE_2D, tex_);

        // glTexImage2D (GL_TEXTURE_2D,
        //               0,
        //               GL_RGBA,
        //               100,
        //               1,
        //               0,
        //               GL_RGBA,
        //               GL_FLOAT,
        //               colors[0].data());

        // glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        // glPushMatrix();
        // glTranslatef(0.0, 0.0, bb[0][2]-0.05f);
        // glBegin(GL_QUADS);
        // glVertex2f(bb[0][0], bb[0][1]);
        // glVertex2f(bb[1][0], bb[0][1]);
        // glVertex2f(bb[1][0], bb[1][1]);
        // glVertex2f(bb[0][0], bb[1][1]);
        // glEnd();
        // glPopMatrix();

        if(net)
        {
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
            net->draw_network();
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
                int x = Fl::event_x();
                int y = Fl::event_y();
                float fx =   2.0f*x/(w()-1) - 1.0f;
                float fy = -(2.0f*y/(h()-1) - 1.0f);
                if(Fl::event_button() == FL_LEFT_MOUSE)
                {
                    nav.get_click(fx, fy);
                }
                else if(Fl::event_button() == FL_RIGHT_MOUSE)
                {
                }
                else if(Fl::event_button() == FL_MIDDLE_MOUSE)
                {
                }
                lastmouse[0] = fx;
                lastmouse[1] = fy;
                redraw();
            }
            take_focus();
            return 1;
        case FL_DRAG:
            {
                int x = Fl::event_x();
                int y = Fl::event_y();
                float fx =  2.0f*x/(w()-1)-1.0f;
                float fy = -(2.0f*y/(h()-1)-1.0f);
                if(Fl::event_button() == FL_LEFT_MOUSE)
                {
                    nav.get_click(fx, fy, 1.0f, true);
                }
                else if(Fl::event_button() == FL_RIGHT_MOUSE)
                {
                    float scale = std::pow(2.0f, zoom-1.0f);

                    double update[3] = {
                        (fx-lastmouse[0])*scale,
                        (fy-lastmouse[1])*scale,
                        0.0f
                    };

                    nav.translate(update);
                }
                else if(Fl::event_button() == FL_MIDDLE_MOUSE)
                {
                    float scale = std::pow(1.5f, zoom-1.0f);
                    zoom += scale*(fy-lastmouse[1]);
                    if(zoom > 17.0f)
                        zoom = 17.0f;
                    else if(zoom < FLT_MIN)
                        zoom = FLT_MIN;

                }
                lastmouse[0] = fx;
                lastmouse[1] = fy;
                redraw();
            }
            take_focus();
            return 1;
        case FL_KEYBOARD:
            switch(Fl::event_key())
            {
            default:
                break;
            }
            redraw();
            return 1;
        case FL_MOUSEWHEEL:
            take_focus();
            return 1;
        default:
            // pass other events to the base class...
            return Fl_Gl_Window::handle(event);
        }
    }

    arcball nav;
    float zoom;
    float lastmouse[2];

    osm::network *net;

    vec3f bb[2];

    GLuint glew_state;
    GLuint tex_;
};
int main(int argc, char *argv[])
{
    std::cerr << libroad_package_string() << std::endl;
    if(argc < 3)
    {
        std::cerr << "Usage: " << argv[0] << " <input osm> <input image>" << std::endl;
        return 1;
    }

    //Load from file
    osm::network net(osm::load_xml_network(argv[1]));
    net.populate_edges_from_hash();
    net.remove_duplicate_nodes();
    net.compute_node_degrees();
    net.clip_roads_to_bounds();
    net.compute_edge_types();
    net.compute_node_degrees();
    net.scale_and_translate();
    net.split_into_road_segments();
    net.remove_highway_intersections();
    net.compute_node_heights();
    net.join_logical_roads();

    Magick::Image im(argv[2]);

    im.quantizeColorSpace(Magick::GRAYColorspace);
    im.quantize();

    im.flip();

    vec2i dim(im.columns(), im.rows());

    float *pix = new float[dim[0]*dim[1]];
    im.write(0, 0, dim[1], dim[0], "R", Magick::FloatPixel, pix);

    im_heightfield ih(pix, dim, 0, 24);
    ih.origin  = vec2f(-50.0);
    ih.spacing = vec2f(1);

    BOOST_FOREACH(osm::edge &e, net.edges)
    {
        ih.displace_shapes(e.shape, 0.1, net);
    }

    delete[] pix;

    // net.create_ramps();
    // net.remove_small_roads(50);
    // net.create_intersections();
    // net.populate_edge_hash_from_edges();

    fltkview mv(0, 0, 500, 500, "fltk View");
    mv.net = &net;

    mv.take_focus();
    Fl::visual(FL_DOUBLE|FL_DEPTH);

    mv.show(1, argv);


    return Fl::run();
}
