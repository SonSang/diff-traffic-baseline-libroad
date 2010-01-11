#include <FL/Fl.H>
#include <FL/Fl_Gl_Window.H>
#include <FL/Fl_Menu_Button.H>
#include <FL/gl.h>
#include <FL/glu.h>
#include "FL/glut.h"
#include "arcball.hpp"
#include "libroad/hwm_network.hpp"

static const float PT_SIZE = 2.0f;

static inline tvmet::XprVector<tvmet::VectorConstReference<float, 3>, 3> cvec3f(const float *mem)
{
    return tvmet::cvector_ref<float,3>(mem);
}

static const float LANE_WIDTH = 2.5f;
static const float CAR_LENGTH = 4.5f;
//* This is the position of the car's axle from the FRONT bumper of the car
static const float CAR_REAR_AXLE = 3.5f;

static GLuint init_draw_car()
{
    static const float verts[][3] = {{-(CAR_LENGTH-CAR_REAR_AXLE), -0.3f*LANE_WIDTH, 0.0f},  //0
                                     {              CAR_REAR_AXLE,-0.15f*LANE_WIDTH, 0.0f},  //1
                                     {              CAR_REAR_AXLE, 0.15f*LANE_WIDTH, 0.0f},  //2
                                     {-(CAR_LENGTH-CAR_REAR_AXLE),  0.3f*LANE_WIDTH, 0.0f},  //3

                                     {-(CAR_LENGTH-CAR_REAR_AXLE), -0.3f*LANE_WIDTH, 1.5f},  //4
                                     {              CAR_REAR_AXLE,-0.15f*LANE_WIDTH, 1.3f},  //5
                                     {              CAR_REAR_AXLE, 0.15f*LANE_WIDTH, 1.3f},  //6
                                     {-(CAR_LENGTH-CAR_REAR_AXLE),  0.3f*LANE_WIDTH, 1.5f}}; //7

    static const int faces[6][4] = {{ 7, 6, 5, 4}, // bottom
                                    { 0, 1, 2, 3}, // top
                                    { 1, 5, 4, 0}, // left side
                                    { 0, 3, 7, 4}, // back
                                    { 3, 7, 6, 2}, // right side
                                    { 5, 6, 2, 1}};// front

    GLuint car_list = glGenLists(1);
    glPushMatrix();
    glNewList(car_list, GL_COMPILE);
    glBegin(GL_QUADS);
    for(int i = 0; i < 6; ++i)
    {
        const vec3f d10 (cvec3f(verts[faces[i][1]]) - cvec3f(verts[faces[i][0]]));
        const vec3f d20 (cvec3f(verts[faces[i][2]]) - cvec3f(verts[faces[i][0]]));
        vec3f norm(tvmet::cross(d10, d20));
        float norm_len = std::sqrt(tvmet::dot(norm, norm));
        norm /= norm_len;
        glNormal3f(norm[0], norm[1], norm[2]);
        for(int j = 0; j < 4; ++j)
            glVertex3fv(&(verts[faces[i][j]][0]));
    }
    glEnd();
    glEndList();
    glPopMatrix();

    return car_list;
}

static void draw_car()
{
    static GLuint car_list = 0;

    if(!car_list)
        car_list = init_draw_car();
    glCallList(car_list);
}

static bool rm_invert(double A[16], double Ainv[16])
{
    double swap_space[4];

    for(int i = 0; i < 4; ++i)
    {
        int maxi = i;
        for(int j = i+1; j < 4; ++j)
            if(std::abs(A[4*j + i]) > std::abs(A[4*maxi + i]))
                maxi = j;
        if(A[4*maxi + i] == 0.0f)
            return false;
        if(i != maxi)
        {
            memcpy(swap_space,       &(A[4*maxi+ 0]), sizeof(double)*4);
            memcpy(&(A[4*maxi + 0]), &(A[4*i   + 0]), sizeof(double)*4);
            memcpy(&(A[4*i + 0]),    swap_space,      sizeof(double)*4);

            memcpy(swap_space,          &(Ainv[4*maxi + 0]), sizeof(double)*4);
            memcpy(&(Ainv[4*maxi + 0]), &(Ainv[4*i    + 0]), sizeof(double)*4);
            memcpy(&(Ainv[4*i    + 0]), swap_space,          sizeof(double)*4);
        }

        double iinv = 1.0f/A[4*i + i];
        A[4*i + i] = 1.0f;
        for(int j = i+1; j < 4; ++j)
            A[4*i + j] *= iinv;
        for(int j = 0; j < 4; ++j)
            Ainv[4*i + j] *= iinv;

        for(int j = 0; j < 4; ++j)
            if(A[4*j + i] != 0.0f && i != j)
            {
                double temp = A[4*j + i];
                A[4*j + i] = 0.0f;
                for(int k = i+1; k < 4; ++k)
                    A[4*j + k] -= A[4*i + k]*temp;
                for(int k = 0; k < 4; ++k)
                    Ainv[4*j + k] -= Ainv[4*i + k]*temp;
            }
    }

    return true;
}

static void pick_ray(vec3f &origin, vec3f &dir, const float x, const float y, const int width, const int height, bool ortho = false)
{
    GLint view[4];
    glGetIntegerv(GL_VIEWPORT, view);
    GLdouble project[16];
    glGetDoublev(GL_PROJECTION_MATRIX, project);
    GLdouble model[16];
    glGetDoublev(GL_MODELVIEW_MATRIX, model);

    double pt[3];
    double o[3];

    GLdouble model2[16];
    GLdouble modelinv[16];

    for(int j = 0; j < 4; ++j)
        for(int i = 0; i < 4; ++i)
            model2[4*j+i] = model[4*i+j];

    memset(modelinv, 0, 16*sizeof(double));
    modelinv[4*0 + 0] = 1.0;
    modelinv[4*1 + 1] = 1.0;
    modelinv[4*2 + 2] = 1.0;
    modelinv[4*3 + 3] = 1.0;

    rm_invert(model2, modelinv);

    gluUnProject(x, height-y, 1.0, model, project, view, pt, pt+1, pt+2);

    if(ortho)
        gluUnProject(x, height-y, -1.0, model, project, view, o, o+1, o+2);
    else
    {
        o[0] = modelinv[4*0 + 3];
        o[1] = modelinv[4*1 + 3];
        o[2] = modelinv[4*2 + 3];
    }

    double v[3] = {pt[0]-o[0], pt[1]-o[1], pt[2]-o[2]};
    double len = 1.0/std::sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
    v[0] *= len;
    v[1] *= len;
    v[2] *= len;

    for(int i = 0; i < 3; ++i)
    {
        origin[i] = o[i];
        dir[i] = v[i];
    }
}

static float ray_point_project(const vec3f &origin, const vec3f &dir, const vec3f &pt)
{
    // assumes dir is unit, so no divide by tvmet::dot(dir, dir);
    return tvmet::dot(dir, pt-origin);
}

static vec3f ray_point_nearest(const vec3f &origin, const vec3f &dir, const vec3f &pt)
{
    return vec3f(origin + dir*ray_point_project(origin, dir, pt));
}

static float ray_point_distance2(const vec3f &origin, const vec3f &dir, const vec3f &pt)
{
    const vec3f diff(pt - ray_point_nearest(origin, dir, pt));
    return tvmet::dot(diff, diff);
}

static float ray_plane_intersection_param(const vec3f &origin, const vec3f &dir, const vec3f &normal, const float support)
{
    const float denom = tvmet::dot(dir, normal);
    assert(std::abs(denom) > 1e-8);

    return (-tvmet::dot(origin, normal) + support)/denom;
}

static vec3f ray_plane_intersection(const vec3f &origin, const vec3f &dir, const vec3f &normal, const float support)
{
    return vec3f(origin + dir*ray_plane_intersection_param(origin, dir, normal, support));
}

class fltkview : public Fl_Gl_Window
{
public:
    fltkview(int x, int y, int w, int h, const char *l) : Fl_Gl_Window(x, y, w, h, l),
                                                          zoom(2.0),
                                                          car_pos(0.0f),
                                                          pick_vert(-1)
    {
        lastmouse[0] = 0.0f;
        lastmouse[1] = 0.0f;

        this->resizable(this);
    }

    void setup_light()
    {
        static const GLfloat light_position[] = { 0.0, 100.0, 0.0, 1.0 };
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
        glLightfv(GL_LIGHT0, GL_POSITION, light_position);
        glPopMatrix();
        glLightfv(GL_LIGHT0, GL_AMBIENT, amb_light_rgba );
        glLightfv(GL_LIGHT0, GL_DIFFUSE, diff_light_rgba );
        glLightfv(GL_LIGHT0, GL_SPECULAR, spec_light_rgba );
        glMaterialfv( GL_FRONT, GL_AMBIENT, material );
        glMaterialfv( GL_FRONT, GL_DIFFUSE, material );
        glMaterialfv( GL_FRONT, GL_SPECULAR, spec_material );
        glMaterialfv( GL_FRONT, GL_SHININESS, &shininess);
    }

    void draw()
    {
        if (!valid())
        {
            glViewport(0, 0, w(), h());
            glMatrixMode(GL_PROJECTION);
            glLoadIdentity();
            gluPerspective(45.0f, (GLfloat)w()/(GLfloat)h(), 10.0f, 500.0f);

            glMatrixMode(GL_MODELVIEW);
            glClearColor(0.0, 0.0, 0.0, 0.0);

            glEnable(GL_DEPTH_TEST);

            glHint(GL_LINE_SMOOTH_HINT,    GL_NICEST);
            glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);

            glEnable(GL_LINE_SMOOTH);

            setup_light();
        }

        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

        glLoadIdentity();

        glTranslatef(0.0f, 0.0f, -std::pow(2.0f, zoom));

        nav.get_rotation();
        glMultMatrixd(nav.world_mat());

        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        glDisable(GL_LIGHTING);

        if(net)
        {
            typedef std::pair<const str, hwm::lane> lmap_itr;
            BOOST_FOREACH(const lmap_itr &l, net->lanes)
            {
                const hwm::lane &la = l.second;
                glColor3f(0.3, 0.3, 0.3);
                glBegin(GL_LINE_LOOP);
                typedef hwm::lane::road_membership::intervals::entry rme;
                BOOST_FOREACH(const rme &rm_entry, la.road_memberships)
                {
                    const hwm::lane::road_membership &rm = rm_entry.second;
                    const std::vector<vec3f> pts(rm.parent_road->rep.extract_center(rm.interval, rm.lane_position-LANE_WIDTH*0.5, 0.5f));
                    BOOST_FOREACH(const vec3f &p, pts)
                    {
                        glVertex3fv(p.data());
                    }
                }
                typedef hwm::lane::road_membership::intervals::const_reverse_iterator rme_it;
                for(rme_it current = la.road_memberships.rbegin(); current != la.road_memberships.rend(); ++current)
                {
                    const hwm::lane::road_membership &rm = current->second;
                    const vec2f rev_interval(rm.interval[1], rm.interval[0]);
                    const std::vector<vec3f> pts(rm.parent_road->rep.extract_center(rev_interval, rm.lane_position+LANE_WIDTH*0.5, 0.5f));
                    BOOST_FOREACH(const vec3f &p, pts)
                    {
                        glVertex3fv(p.data());
                    }
                }
                glEnd();

                glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
                glEnable(GL_LIGHTING);

                {
                    mat4x4f trans(la.point_frame(car_pos));
                    mat4x4f ttrans(tvmet::trans(trans));
                    glColor3f(1.0, 1.0, 0.0);

                    glDisable(GL_BLEND);
                    glEnable(GL_LIGHTING);
                    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

                    glPushMatrix();
                    glMultMatrixf(ttrans.data());
                    draw_car();
                    glPopMatrix();
                }

                glEnable(GL_BLEND);
                glDisable(GL_LIGHTING);
            }
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

    float         car_pos;
    hwm::network *net;

    int pick_vert;
};

int main(int argc, char *argv[])
{
    hwm::network net(hwm::load_xml_network(argv[1]));
    net.scale_offsets(LANE_WIDTH);

    std::cerr << "HWM net loaded successfully" << std::endl;

    if(net.check())
        std::cerr << "HWM net checks out" << std::endl;
    else
        std::cerr << "HWM net doesn't check out" << std::endl;

    fltkview mv(0, 0, 500, 500, "fltk View");

    mv.net = &net;

    mv.take_focus();
    Fl::visual(FL_DOUBLE|FL_DEPTH);

    mv.show(1, argv);
    return Fl::run();
}
