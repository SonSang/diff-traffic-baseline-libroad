#include <GL/glew.h>
#include <FL/Fl.H>
#include <FL/Fl_Gl_Window.H>
#include <FL/Fl_Menu_Button.H>
#include <FL/gl.h>
#include <FL/glu.h>
#include "FL/glut.h"
#include "arcball.hpp"
#include "libroad/arc_road.hpp"
#include "libroad/hwm_draw.hpp"

static const float PT_SIZE       = 4.0f;
static const float LANE_WIDTH    = 2.5f;
static const float CAR_LENGTH    = 4.5f;
//* This is the position of the car's axle from the FRONT bumper of the car
static const float CAR_REAR_AXLE = 3.5f;

static const char vertexShader[] =
{
    "void main()                                                            \n"
    "{                                                                      \n"
    "    float pointSize = 500.0 * gl_Point.size;                           \n"
	"    vec4 vert = gl_Vertex;												\n"
	"    vert.w = 1.0;														\n"
    "    vec3 pos_eye = vec3 (gl_ModelViewMatrix * vert);                   \n"
    "    gl_PointSize = max(1.0, pointSize / (1.0 - pos_eye.z));            \n"
    "    gl_TexCoord[0] = gl_MultiTexCoord0;                                \n"
    "    gl_Position = ftransform();                                        \n"
    "    gl_FrontColor = gl_Color;                                          \n"
    "}                                                                      \n"
};

static const char pixelShader[] =
{
    "uniform sampler2D splatTexture;                                        \n"

    "void main()                                                            \n"
    "{                                                                      \n"
    "    vec4 color =  gl_Color * texture2D(splatTexture, gl_TexCoord[0].st); \n"
    "    gl_FragColor =                                                     \n"
    "         color;\n"
    "}                                                                      \n"
};

static unsigned char* mytex(int N)
{
    unsigned char *B = new unsigned char[4*N*N];

    const float inc = 2.0f/(N-1);

    float y = -1.0f;
    for (int j=0; j < N; j++)
    {
        float x = -1.0f;
        for (int i=0; i < N; i++)
        {
            unsigned char v =  (x*x + y*y) > 1.0f ? 0 : 255;
            int idx = (j*N + i)*4;
            B[idx+3] = B[idx+2] = B[idx+1] = B[idx] = v;
            x += inc;
        }
        y += inc;
    }
    return B;
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

        const double iinv = 1.0f/A[4*i + i];
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

    vec3d pt;
    vec3d o;

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

    gluUnProject(x, height-y, 1.0, model, project, view, pt.data(), pt.data()+1, pt.data()+2);

    if(ortho)
        gluUnProject(x, height-y, -1.0, model, project, view, o.data(), o.data()+1, o.data()+2);
    else
    {
        o[0] = modelinv[4*0 + 3];
        o[1] = modelinv[4*1 + 3];
        o[2] = modelinv[4*2 + 3];
    }

    const vec3d v(tvmet::normalize(pt - o));
    for(int i = 0; i < 3; ++i)
    {
        origin[i] = o[i];
        dir[i]    = v[i];
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
                                                          low_bnd(0.0f), high_bnd(0.0f), low_res(0.3f), high_res(0.3f),
                                                          car_pos(0.0f),
                                                          glew_state(GLEW_OK+1),
                                                          vertex_shader(0), pixel_shader(0), program(0), texture(0), num_tex(0), pick_vert(-1), tex_(0)
    {
        lastmouse[0] = 0.0f;
        lastmouse[1] = 0.0f;
        extract[0] = 0.0f;
        extract[1] = 1.0f;

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

    void init_shaders()
    {
        vertex_shader = glCreateShader(GL_VERTEX_SHADER);
        pixel_shader  = glCreateShader(GL_FRAGMENT_SHADER);

        const char* v = vertexShader;
        const char* p = pixelShader;
        glShaderSource(vertex_shader, 1, &v, 0);
        glShaderSource(pixel_shader, 1, &p, 0);

        glCompileShader(vertex_shader);
        glCompileShader(pixel_shader);

        program = glCreateProgram();

        glAttachShader(program, vertex_shader);
        glAttachShader(program, pixel_shader);

        glLinkProgram(program);
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

    void make_texture()
    {
        int resolution = 32;
        unsigned char *data = mytex(resolution);

        glGenTextures(1, &texture);
        glBindTexture(GL_TEXTURE_2D, texture);
        glTexParameteri(GL_TEXTURE_2D, GL_GENERATE_MIPMAP_SGIS, GL_TRUE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, resolution, resolution, 0,
                     GL_RGBA, GL_UNSIGNED_BYTE, data);

        delete data;

        glGenTextures(1, &num_tex);
        glBindTexture(GL_TEXTURE_RECTANGLE_ARB, num_tex);
        glTexParameteri(GL_TEXTURE_2D, GL_GENERATE_MIPMAP_SGIS, GL_TRUE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
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

            if(GLEW_OK != glew_state)
                init_glew();

            if(!program)
                init_shaders();

            if(!texture)
                make_texture();

            if(!car_drawer.initialized())
                car_drawer.initialize(0.6*LANE_WIDTH,
                                      CAR_LENGTH,
                                      1.5f,
                                      CAR_REAR_AXLE);

            setup_light();
        }

        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

        glLoadIdentity();

        glTranslatef(0.0f, 0.0f, -std::pow(2.0f, zoom));

        nav.get_rotation();
        glMultMatrixd(nav.world_mat());

        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

        if(ar)
        {
            glColor3f(0.3, 0.3, 0.3);
            glBegin(GL_LINE_STRIP);
            BOOST_FOREACH(const vec3f &p, ar->points_)
            {
                glVertex3fv(p.data());
            }
            glEnd();

            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
            glColor3f(1.0, 0.0, 0.0);
            glBegin(GL_LINE_STRIP);
            {
                std::vector<vertex> pts;
                ar->extract_center(pts, extract, low_bnd, low_res);
                BOOST_FOREACH(const vertex &p, pts)
                {
                    glVertex3fv(p.position.data());
                }
            }
            glEnd();
            glColor3f(0.0, 1.0, 0.0);
            glBegin(GL_LINE_STRIP);
            {
                std::vector<vertex> pts;
                ar->extract_center(pts, extract, high_bnd, high_res);
                BOOST_FOREACH(const vertex &p, pts)
                {
                    glVertex3fv(p.position.data());
                }
            }
            glEnd();

            std::vector<vertex> v;
            std::vector<vec3u> f;
            ar->make_mesh(v, f, vec2f(0.0, 1.0), vec2f(-0.5, 0.5), 2.0);
            glBegin(GL_TRIANGLES);
            BOOST_FOREACH(const vec3u &fa, f)
            {
                BOOST_FOREACH(const unsigned int &idx, fa)
                {
                    glTexCoord2fv(v[idx].tex_coord.data());
                    glNormal3fv(v[idx].normal.data());
                    glVertex3fv(v[idx].position.data());
                }
            }
            glEnd();

            {
                mat4x4f trans(ar->point_frame(car_pos, low_bnd, false));
                mat4x4f ttrans(tvmet::trans(trans));
                glColor3f(1.0, 1.0, 0.0);

                glDisable(GL_BLEND);
                glEnable(GL_LIGHTING);
                glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

                glPushMatrix();

                glMultMatrixf(ttrans.data());
                car_drawer.draw();
                glPopMatrix();

                glEnable(GL_BLEND);
                glDisable(GL_LIGHTING);
            }
            {
                mat4x4f trans(ar->point_frame(extract[0], 0.0f, false));
                mat4x4f ttrans(tvmet::trans(trans));
                glColor3f(1.0, 0.0, 0.0);

                glDisable(GL_BLEND);
                glEnable(GL_LIGHTING);
                glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

                glPushMatrix();

                glMultMatrixf(ttrans.data());
                car_drawer.draw();
                glPopMatrix();

                glEnable(GL_BLEND);
                glDisable(GL_LIGHTING);
            }

            {
                mat4x4f trans(ar->point_frame(extract[1], 0.0f, false));
                mat4x4f ttrans(tvmet::trans(trans));
                glColor3f(0.0, 0.0, 1.0);

                glDisable(GL_BLEND);
                glEnable(GL_LIGHTING);
                glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

                glPushMatrix();

                glMultMatrixf(ttrans.data());
                car_drawer.draw();
                glPopMatrix();

                glEnable(GL_BLEND);
                glDisable(GL_LIGHTING);
            }

            if(pick_vert != -1 && (Fl::event_state() & FL_SHIFT) && (Fl::event_state() & FL_CTRL))
            {
                glColor3f(1.0, 1.0, 1.0);
                glBegin(GL_LINES);
                glVertex3f(ar->points_[pick_vert][0], ar->points_[pick_vert][1], -1000.0f);
                glVertex3f(ar->points_[pick_vert][0], ar->points_[pick_vert][1],  1000.0f);
                glEnd();
            }

            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

            glEnable(GL_TEXTURE_2D);

            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            glEnable(GL_BLEND);

            // setup point sprites
            glEnable(GL_POINT_SPRITE_ARB);
            glTexEnvi(GL_POINT_SPRITE_ARB, GL_COORD_REPLACE_ARB, GL_TRUE);
            glEnable(GL_VERTEX_PROGRAM_POINT_SIZE_NV);
            glPointSize(PT_SIZE);

            glUseProgram(program);
            GLuint texLoc = glGetUniformLocation(program, "splatTexture");
            glUniform1i(texLoc, 0);

            glActiveTextureARB(GL_TEXTURE0_ARB);
            glBindTexture(GL_TEXTURE_2D, texture);

            glDepthMask(GL_FALSE);

            glBegin(GL_POINTS);
            for(size_t i = 0; i < ar->points_.size(); ++i)
            {
                if(static_cast<int>(i) == pick_vert)
                    glColor3f(0.0, 1.0, 1.0);
                else
                    glColor3f(0.0, 0.0, 1.0);
                glVertex3fv(ar->points_[i].data());
            }
            glEnd();

            glUseProgram(0);
            glDisable(GL_POINT_SPRITE_ARB);
            glDisable(GL_TEXTURE_2D);

            glDepthMask(GL_TRUE);
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
                    if(Fl::event_state() & FL_SHIFT)
                    {
                        vec3f origin;
                        vec3f dir;
                        pick_ray(origin, dir, x, y, w(), h());

                        size_t min_pt;
                        float min_dist = FLT_MAX;
                        for(size_t i = 0; i < ar->points_.size(); ++i)
                        {
                            const float dist2(ray_point_distance2(origin, dir, ar->points_[i]));
                            if(dist2 < min_dist)
                            {
                                min_pt = i;
                                min_dist = dist2;
                            }
                        }

                        if(min_dist < PT_SIZE*PT_SIZE)
                            pick_vert = min_pt;
                        else
                            pick_vert = -1;
                    }
                    else
                    {
                        nav.get_click(fx, fy);
                    }
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
                    if(Fl::event_state() & FL_SHIFT)
                    {
                        if(pick_vert != -1)
                        {
                            if(Fl::event_state() & FL_CTRL)
                            {
                                vec3f origin;
                                vec3f dir;
                                pick_ray(origin, dir, w()/2, h()/2, w(), h());
                                const float fac = 1.0f - tvmet::dot(dir, vec3f(0.0f, 0.0f, 1.0f));
                                const float scale = std::pow(2.0f, zoom-1.0f);
                                ar->points_[pick_vert][2] += fac*(fy-lastmouse[1])*scale;
                            }
                            else
                            {
                                vec3f origin;
                                vec3f dir;
                                pick_ray(origin, dir, x, y, w(), h());
                                const vec3f inters(ray_plane_intersection(origin, dir, vec3f(0.0, 0.0, 1.0), ar->points_[pick_vert][2]));
                                ar->points_[pick_vert] = inters;
                            }

                            ar->initialize();
                        }
                    }
                    else
                    {
                        nav.get_click(fx, fy, 1.0f, true);
                    }
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
            case 'q':
                low_bnd -= 0.1;
                break;
            case 'w':
                low_bnd += 0.1;
                break;
            case 'e':
                high_bnd -= 0.1;
                break;
            case 'r':
                high_bnd += 0.1;
                break;
            case 'a':
                low_res *= 0.5;
                break;
            case 's':
                low_res *= 2.0;
                break;
            case 'd':
                high_res *= 0.5;
                break;
            case 'f':
                high_res *= 2.0;
                break;
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
            case '1':
                extract[0] -= 0.02f;
                if(extract[0] < 0.0f)
                    extract[0] = 0.0f;
                break;
            case '2':
                extract[0] += 0.02f;
                if(extract[0] > 1.0f)
                    extract[0] = 1.0f;
                break;
            case '3':
                extract[1] -= 0.02f;
                if(extract[1] < 0.0f)
                    extract[1] = 0.0f;
                break;
            case '4':
                extract[1] += 0.02f;
                if(extract[1] > 1.0f)
                    extract[1] = 1.0f;
                break;
            case 'l':
                if(ar)
                {
                    std::cout << "Lengths:\n"
                              << "Original: " << ar->length(0) << '\n'
                              << "Red:      " << ar->length(low_bnd) << '\n'
                              << "Green:    " << ar->length(high_bnd) << std::endl;

                }
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

    float low_bnd;
    float high_bnd;
    float low_res;
    float high_res;

    float     car_pos;
    arc_road *ar;

    GLuint glew_state;

    GLuint vertex_shader;
    GLuint pixel_shader;
    GLuint program;
    GLuint texture;
    GLuint num_tex;

    int pick_vert;

    vec2f         extract;
    hwm::car_draw car_drawer;
    GLuint        tex_;
};

int main(int argc, char *argv[])
{
    arc_road ar;
    ar.points_.push_back(vec3f(00.0, 40.0, 0.0));
    ar.points_.push_back(vec3f(40.0, 30.0, 0.0));
    ar.points_.push_back(vec3f(40.0, 00.0, 0.0));
    ar.points_.push_back(vec3f(60.0, 00.0, 0.0));
    ar.points_.push_back(vec3f(30.0, -20.0, 0.0));
    ar.points_.push_back(vec3f(20.0, -10.0, 1.0));
    ar.points_.push_back(vec3f(20.0, -40.0, 1.0));
    ar.points_.push_back(vec3f(00.5, -20.0, 1.0));
    ar.points_.push_back(vec3f(00.5, -50.0, 2.0));
    ar.points_.push_back(vec3f(10.0, -70.0, 2.0));
    ar.points_.push_back(vec3f(00.0, -90, 2.0));
    ar.points_.push_back(vec3f(50.0, -80, 3.0));
    ar.points_.push_back(vec3f(70.5, -50, 3.0));
    ar.points_.push_back(vec3f(100, - 30, 3.0));
    ar.initialize();

    fltkview mv(0, 0, 500, 500, "fltk View");

    mv.ar = &ar;

    mv.take_focus();
    Fl::visual(FL_DOUBLE|FL_DEPTH);

    mv.show(1, argv);
    return Fl::run();
}
