#include <Magick++.h>
#include <GL/glew.h>
#include <FL/Fl.H>
#include <FL/Fl_Gl_Window.H>
#include <FL/Fl_Menu_Button.H>
#include <FL/gl.h>
#include <FL/glu.h>
#include "FL/glut.h"
#include "arcball.hpp"
#include "geometric.hpp"
#include "im_heightfield.hpp"

#include <fstream>

class fltkview : public Fl_Gl_Window
{
public:
    fltkview(int x, int y, int w, int h, const char *l) : Fl_Gl_Window(x, y, w, h, l),
                                                          zoom(2.0),
                                                          lastmouse(0),
                                                          lastpick(0),
                                                          ih(0),
                                                          glew_state(GLEW_OK+1),
                                                          tex_(0),
                                                          meshlist(0),
                                                          do_height(0),
                                                          noise_scale(1),
                                                          noise_origin(0),
                                                          octaves(10),
                                                          power(1.0)
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
            glEnable(GL_TEXTURE_2D);
            glBindTexture (GL_TEXTURE_2D, tex_);
            glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
            // glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            // glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
            glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
            glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
            retex();
            glDisable(GL_TEXTURE_2D);
        }
    }

    void gen_h_list()
    {
        if(meshlist)
            glDeleteLists(meshlist, 1);

        meshlist = glGenLists(1);

        glPushMatrix();
        glLoadIdentity();
        glNewList(meshlist, GL_COMPILE);
        std::vector<vertex> vrts;
        std::vector<vec3u> faces;
        ih->make_mesh(vrts, faces, false);
        glBegin(GL_TRIANGLES);
        BOOST_FOREACH(const vec3u &face, faces)
        {
            BOOST_FOREACH(const unsigned int idx, face)
            {
                glTexCoord2fv(vrts[idx].tex_coord.data());
                glVertex3fv(vrts[idx].position.data());
            }
        }
        glEnd();
        glEndList();
        glPopMatrix();
    }

    void retex()
    {
        ih->set_turbulence(vec2f(-noise_origin), 1.0f/noise_scale, 0, octaves, power);

        glTexImage2D (GL_TEXTURE_2D,
                      0,
                      1,
                      ih->dim[0],
                      ih->dim[1],
                      0,
                      GL_LUMINANCE,
                      GL_FLOAT,
                      ih->pix);

        if(do_height)
            gen_h_list();
    }

    void draw()
    {
        if (!valid())
        {
            glViewport(0, 0, w(), h());
            glMatrixMode(GL_PROJECTION);
            glLoadIdentity();
            gluPerspective(45.0f, (GLfloat)w()/(GLfloat)h(), 30.0f, 50000.0f);

            glMatrixMode(GL_MODELVIEW);
            glClearColor(0.0, 0.0, 0.0, 0.0);

            glEnable(GL_DEPTH_TEST);

            glHint(GL_LINE_SMOOTH_HINT,    GL_NICEST);
            glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);

            glEnable(GL_LINE_SMOOTH);

            if(GLEW_OK != glew_state)
                init_glew();

            if(ih)
            {
                init_textures();
            }
        }

        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

        glLoadIdentity();

        glTranslatef(0.0f, 0.0f, -std::pow(2.0f, zoom));

        nav.get_rotation();
        glMultMatrixd(nav.world_mat());

        if(ih)
        {
            glEnable(GL_TEXTURE_2D);
            glBindTexture (GL_TEXTURE_2D, tex_);

            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
            glPushMatrix();
            glTranslatef(ih->origin[0], ih->origin[1], ih->zbase);
            glScalef(ih->spacing[0], ih->spacing[1], ih->zscale);

            if(do_height)
                glCallList(meshlist);
            else
            {
                glScalef(ih->dim[0], ih->dim[1], 1);
                glBegin(GL_QUADS);
                glTexCoord2f(0.0, 0.0);
                glVertex2f(0.0, 0.0);
                glTexCoord2f(1.0, 0.0);
                glVertex2f(1.0, 0.0);
                glTexCoord2f(1.0, 1.0);
                glVertex2f(1.0, 1.0);
                glTexCoord2f(0.0, 1.0);
                glVertex2f(0.0, 1.0);
                glEnd();
            }
            glPopMatrix();

            glDisable(GL_TEXTURE_2D);
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
                    if(Fl::event_state() & FL_SHIFT && ih)
                    {
                        vec3f origin;
                        vec3f dir;
                        pick_ray(origin, dir, x, y, w(), h());
                        const vec3f inters(ray_plane_intersection(origin, dir, vec3f(0.0, 0.0, 1.0), ih->zbase));
                        lastpick = sub<0,2>::vector(inters);
                    }
                    else
                    {
                        nav.get_click(fx, fy);
                    }
                }
                else if(Fl::event_button() == FL_RIGHT_MOUSE)
                {
                    if(Fl::event_state() & FL_SHIFT && ih)
                    {
                        vec3f origin;
                        vec3f dir;
                        pick_ray(origin, dir, x, y, w(), h());
                        const vec3f inters(ray_plane_intersection(origin, dir, vec3f(0.0, 0.0, 1.0), ih->zbase));
                        lastpick = sub<0,2>::vector(inters);
                    }
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
                    if(Fl::event_state() & FL_SHIFT && ih)
                    {
                        vec3f       origin;
                        vec3f       dir;
                        pick_ray(origin, dir, x, y, w(), h());
                        const vec3f inters(ray_plane_intersection(origin, dir, vec3f(0.0, 0.0, 1.0), ih->zbase));

                        const vec2f diff(sub<0,2>::vector(inters) - lastpick);

                        if(Fl::event_state() & FL_CTRL)
                            ih->spacing *= length(sub<0,2>::vector(inters)-ih->origin)/length(lastpick-ih->origin);
                        else
                            sub<0,2>::vector(ih->origin) += diff;

                        lastpick                      = sub<0,2>::vector(inters);
                    }
                    else
                    {
                        nav.get_click(fx, fy, 1.0f, true);
                    }
                }
                else if(Fl::event_button() == FL_RIGHT_MOUSE)
                {
                    if(Fl::event_state() & FL_SHIFT && ih)
                    {
                        vec3f       origin;
                        vec3f       dir;
                        pick_ray(origin, dir, x, y, w(), h());
                        const vec3f inters(ray_plane_intersection(origin, dir, vec3f(0.0, 0.0, 1.0), ih->zbase));

                        const vec2f diff(sub<0,2>::vector(inters) - lastpick);

                        if(Fl::event_state() & FL_CTRL)
                            noise_scale *= length(sub<0,2>::vector(inters)-ih->origin)/length(lastpick-ih->origin);
                        else
                            sub<0,2>::vector(noise_origin) += diff/noise_scale;
                        retex();
                        lastpick = sub<0,2>::vector(inters);
                    }
                    else
                    {
                        float scale = std::pow(2.0f, zoom-1.0f);

                        double update[3] = {
                            (fx-lastmouse[0])*scale,
                            (fy-lastmouse[1])*scale,
                            0.0f
                        };

                        nav.translate(update);
                    }
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
            case '1':
                {
                    ih->dim /= 2;
                    ih->spacing *= 2;
                    delete[] ih->pix;
                    ih->pix = new float[ih->dim[0]*ih->dim[1]];
                    retex();
                    if(do_height)
                        gen_h_list();
                }
                break;
            case '2':
                {
                    ih->dim *= 2;
                    ih->spacing /= 2;
                    delete[] ih->pix;
                    ih->pix = new float[ih->dim[0]*ih->dim[1]];
                    retex();
                    if(do_height)
                        gen_h_list();
                }
                break;
            case 'h':
                {
                    do_height = !do_height;
                    if(do_height)
                        gen_h_list();
                }
                break;
            case 'w':
                if(ih)
                {
                    std::cerr << "Image: dim " << ih->dim << std::endl
                              << "origin     " << ih->origin << std::endl
                              << "spacing    " << ih->spacing << std::endl
                              << "zbase      " << ih->zbase << std::endl
                              << "zscale     " << ih->zscale << std::endl;

                    Magick::Image im;
                    im.read(ih->dim[0], ih->dim[1], "R", Magick::FloatPixel, ih->pix);
                    im.colorSpace(Magick::GRAYColorspace);
                    im.quantize();
                    im.write(iname);
                    std::cerr << "wrote " << iname << std::endl;
                }
                break;
            case 'd':
                if(ih)
                {
                    std::cerr << "Image: dim " << ih->dim << std::endl
                              << "origin     " << ih->origin << std::endl
                              << "spacing    " << ih->spacing << std::endl
                              << "zbase      " << ih->zbase << std::endl
                              << "zscale     " << ih->zscale << std::endl;

                    std::vector<vertex> vrts;
                    std::vector<vec3u>  fcs;
                    ih->make_mesh(vrts, fcs);
                    std::ofstream os("default.obj");
                    mesh_to_obj(os, "image", "terrain", vrts, fcs);
                    std::cerr << "Wrote default.obj" << std::endl;
                    std::ofstream os2("default.smf");
                    mesh_to_smf(os2, vrts, fcs);
                    std::cerr << "Wrote default.smf" << std::endl;
                }
                break;
            case '9':
                if(ih)
                {
                    ih->zscale *= 0.5;
                }
                break;
            case '0':
                if(ih)
                {
                    ih->zscale *= 2;
                }
                break;
            case 'o':
                if(ih)
                {
                    --octaves;
                    if(octaves < 1)
                        octaves = 1;
                    retex();
                }
                break;
            case 'p':
                if(ih)
                {
                    ++octaves;
                    retex();
                }
                break;
            case ';':
                if(ih)
                {
                    power -= 0.1;
                    retex();
                }
                break;
            case '\'':
                if(ih)
                {
                    power += 0.1;
                    retex();
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
    vec2f lastmouse;

    vec2f lastpick;

    im_heightfield *ih;

    GLuint glew_state;
    GLuint tex_;
    GLuint meshlist;
    bool do_height;

    float       noise_scale;
    vec2f       noise_origin;
    int         octaves;
    float       power;
    std::string iname;
};
int main(int argc, char *argv[])
{
    std::cerr << libroad_package_string() << std::endl;
    if(argc < 3)
    {
        std::cerr << "Usage: " << argv[0] << " <x dim> <y dim> <output image>" << std::endl;
        return 1;
    }

    vec2i dim(atoi(argv[1]), atoi(argv[2]));

    float *pix = new float[dim[0]*dim[1]];

    im_heightfield ih(pix, dim, 0, 10);
    ih.origin  = vec2f(0.0);
    ih.spacing = vec2f(0.1);

    fltkview mv(0, 0, 500, 500, "fltk View");
    mv.ih    = &ih;
    mv.iname = argv[3];

    mv.take_focus();
    Fl::visual(FL_DOUBLE|FL_DEPTH);

    mv.show(1, argv);
    return Fl::run();
}
