#include <GL/glew.h>
#include <FL/Fl.H>
#include <FL/Fl_Gl_Window.H>
#include <FL/Fl_Menu_Button.H>
#include <FL/gl.h>
#include <FL/glu.h>
#include "FL/glut.h"
#include "arcball.hpp"
#include "libroad/hwm_network.hpp"
#include "libroad/hwm_draw.hpp"

static const float LANE_WIDTH = 2.5f;
static const float CAR_LENGTH = 4.5f;
//* This is the position of the car's axle from the FRONT bumper of the car
static const float CAR_REAR_AXLE = 3.5f;

class fltkview : public Fl_Gl_Window
{
public:
    fltkview(int x, int y, int w, int h, const char *l) : Fl_Gl_Window(x, y, w, h, l),
                                                          zoom(2.0),
                                                          car_pos(0.0f),
                                                          pick_vert(-1),
                                                          glew_state(GLEW_OK+1),
                                                          light_position(50.0, 100.0, 50.0, 1.0)
    {
        lastmouse[0] = 0.0f;
        lastmouse[1] = 0.0f;

        this->resizable(this);
    }

    void setup_light()
    {
        static const GLfloat amb_light_rgba[] = { 0.1, 0.1, 0.1, 1.0 };
        static const GLfloat diff_light_rgba[] = { 0.7, 0.7, 0.7, 1.0 };
        static const GLfloat spec_light_rgba[] = { 1.0, 1.0, 1.0, 1.0 };
        static const GLfloat spec_material[] = { 1.0, 1.0, 1.0, 1.0 };
        static const GLfloat material[] = { 1.0, 1.0, 1.0, 1.0 };
        static const GLfloat shininess = 100.0;

        glEnable(GL_LIGHTING);
        glEnable(GL_LIGHT0);
        glEnable(GL_COLOR_MATERIAL);
        glPushMatrix();
        glLoadIdentity();
        glLightfv(GL_LIGHT0, GL_POSITION, light_position.data());
        glPopMatrix();
        glLightfv(GL_LIGHT0, GL_AMBIENT, amb_light_rgba );
        glLightfv(GL_LIGHT0, GL_DIFFUSE, diff_light_rgba );
        glLightfv(GL_LIGHT0, GL_SPECULAR, spec_light_rgba );
        glMaterialfv( GL_FRONT, GL_AMBIENT, material );
        glMaterialfv( GL_FRONT, GL_DIFFUSE, material );
        glMaterialfv( GL_FRONT, GL_SPECULAR, spec_material );
        glMaterialfv( GL_FRONT, GL_SHININESS, &shininess);
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
            glMatrixMode(GL_PROJECTION);
            glLoadIdentity();
            gluPerspective(45.0f, (GLfloat)w()/(GLfloat)h(), 10.0f, 5000.0f);

            glMatrixMode(GL_MODELVIEW);
            glClearColor(0.0, 0.0, 0.0, 0.0);

            glEnable(GL_DEPTH_TEST);

            glHint(GL_LINE_SMOOTH_HINT,    GL_NICEST);
            glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);

            glEnable(GL_LINE_SMOOTH);

            if(GLEW_OK != glew_state)
                init_glew();

            if(!car_drawer.initialized())
                car_drawer.initialize(0.6*LANE_WIDTH,
                                      CAR_LENGTH,
                                      1.5f,
                                      CAR_REAR_AXLE);

            if(!network_drawer.initialized())
                network_drawer.initialize(net, LANE_WIDTH);

            setup_light();

            if(net)
            {
                bb[0] = vec3f(FLT_MAX);
                bb[1] = vec3f(-FLT_MAX);
                net->bounding_box(bb[0], bb[1]);
            }
        }

        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

        glLoadIdentity();

        glTranslatef(0.0f, 0.0f, -std::pow(2.0f, zoom));

        nav.get_rotation();
        glMultMatrixd(nav.world_mat());

        glLightfv(GL_LIGHT0, GL_POSITION, light_position.data());

        if(net)
        {
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
            glDisable(GL_LIGHTING);
            glColor3f(0.1, 0.2, 0.1);
            glPushMatrix();
            glTranslatef(0.0, 0.0, bb[0][2]-0.05f);
            glBegin(GL_QUADS);
            glVertex2f(bb[0][0], bb[0][1]);
            glVertex2f(bb[1][0], bb[0][1]);
            glVertex2f(bb[1][0], bb[1][1]);
            glVertex2f(bb[0][0], bb[1][1]);
            glEnd();
            glPopMatrix();

            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
            glDisable(GL_LIGHTING);

            glColor3f(0.5, 0.5, 0.5);
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
            glEnable(GL_LIGHTING);
            network_drawer.draw_lanes_solid();

            glColor3f(0.5, 0.5, 1.0);
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
            glDisable(GL_LIGHTING);
            network_drawer.draw_intersections_wire();

            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
            glEnable(GL_LIGHTING);

            typedef std::pair<const str, hwm::lane> lmap_itr;
            BOOST_FOREACH(const lmap_itr &l, net->lanes)
            {
                const hwm::lane &la = l.second;

                mat4x4f trans(la.point_frame(car_pos));
                mat4x4f ttrans(tvmet::trans(trans));
                glColor3f(1.0, 1.0, 0.0);

                glDisable(GL_BLEND);
                glEnable(GL_LIGHTING);
                glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

                glPushMatrix();
                glMultMatrixf(ttrans.data());
                car_drawer.draw();
                glPopMatrix();
            }

            glEnable(GL_BLEND);
            glDisable(GL_LIGHTING);
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
            case 't':
                car_pos += 0.02f;
                if(car_pos > 1.0f)
                    car_pos = 1.0f;
                break;
            case 'g':
                car_pos -= 0.02f;
                if(car_pos < 0.0f)
                    car_pos = 0.0f;
                break;
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

    float              car_pos;
    hwm::car_draw      car_drawer;
    hwm::network_draw  network_drawer;
    hwm::network      *net;

    vec3f bb[2];

    int pick_vert;
    GLuint glew_state;
    vec4f light_position;
};

int main(int argc, char *argv[])
{
    hwm::network net(hwm::load_xml_network(argv[1], vec3f(1.0, 1.0, 15.0f)));
    net.scale_offsets(LANE_WIDTH);
    net.build_intersections(LANE_WIDTH);
    net.center();

    std::cerr << "HWM net loaded successfully" << std::endl;

    if(net.check())
        std::cerr << "HWM net checks out" << std::endl;
    else
    {
        std::cerr << "HWM net doesn't check out" << std::endl;
        exit(1);
    }

    fltkview mv(0, 0, 500, 500, "fltk View");

    mv.net = &net;

    mv.take_focus();
    Fl::visual(FL_DOUBLE|FL_DEPTH);

    mv.show(1, argv);
    return Fl::run();
}
