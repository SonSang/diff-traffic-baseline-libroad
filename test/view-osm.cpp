#include <Magick++.h>
#include <GL/glew.h>
#include <FL/Fl.H>
#include <FL/Fl_Gl_Window.H>
#include <FL/Fl_Menu_Button.H>
#include <FL/gl.h>
#include <FL/glu.h>
#include "FL/glut.h"
#include <fstream>

#include "arcball.hpp"
#include "libroad/geometric.hpp"
#include "visual_geometric.hpp"
#include "im_heightfield.hpp"

#include "libroad/osm_network.hpp"
#include "libroad/hwm_network.hpp"

class fltkview : public Fl_Gl_Window
{
public:
    fltkview(int x, int y, int w, int h, const char *l) : Fl_Gl_Window(x, y, w, h, l),
                                                          zoom(2.0),
                                                          lastmouse(0),
                                                          lastpick(0),
                                                          net(0),
                                                          ih(0),
                                                          glew_state(GLEW_OK+1),
                                                          tex_(0),
                                                          meshlist(0),
                                                          do_height(0),
                                                          noise_scale(10.0),
                                                          noise_origin(0.0),
                                                          octaves(3),
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
            glBindTexture (GL_TEXTURE_2D, tex_);
            glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
            // glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            // glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
            glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
            glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
            retex();
        }
    }

    void gen_h_list()
    {
        std::cerr << "Generating heightfield... ";
        std::cerr.flush();

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

        std::cerr << "Done. " << std::endl;
    }

    void retex()
    {
        if(!const_image)
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
        {
            gen_h_list();
            if(net)
                reproject_net();
        }
    }

    void draw()
    {
        if (!valid())
        {
            glViewport(0, 0, w(), h());
            glMatrixMode(GL_PROJECTION);
            glLoadIdentity();
            gluPerspective(45.0f, (GLfloat)w()/(GLfloat)h(), 100.0f, 500000.0f);

            glMatrixMode(GL_MODELVIEW);
            glClearColor(0.0, 0.0, 0.0, 0.0);

            glEnable(GL_DEPTH_TEST);

            glHint(GL_LINE_SMOOTH_HINT,    GL_NICEST);
            glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);

            glEnable(GL_LINE_SMOOTH);

            if(GLEW_OK != glew_state)
                init_glew();

            if(ih)
                init_textures();
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

        if(net)
        {
            glTranslatef(0.0, 0.0, 0.01f);
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
            net->draw_network();
        }

        glFlush();
        glFinish();
    }

    void reproject_net()
    {
        if(net && ih && do_height)
        {
            std::cerr << "Projecting on to heightfield... ";
            std::cerr.flush();
            BOOST_FOREACH(osm::edge &e, net->edges)
            {
                ih->displace_shapes(e.shape, 5.0, 0.1, *net);
            }
            std::cerr << " Done. " << std::endl;
        }
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
                        if(do_height)
                            reproject_net();
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
                            sub<0,2>::vector(noise_origin) += diff/(ih->spacing[0]*noise_scale);
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
                    ih->dim      = (ih->dim-1)/2 + 1;
                    ih->spacing *= 2;
                    noise_scale /= 2;
                    delete[] ih->pix;
                    ih->pix      = new float[ih->dim[0]*ih->dim[1]];
                    retex();
                    if(do_height)
                        gen_h_list();
                }
                break;
            case '2':
                {
                    ih->dim      = (ih->dim-1)*2 + 1;
                    ih->spacing /= 2;
                    noise_scale *= 2;
                    delete[] ih->pix;
                    ih->pix      = new float[ih->dim[0]*ih->dim[1]];
                    retex();
                    if(do_height)
                        gen_h_list();
                }
                break;
            case 'h':
                {
                    do_height = !do_height;
                    if(do_height)
                    {
                        gen_h_list();
                        reproject_net();
                    }
                }
                break;
            case 'w':
                if(ih)
                {
                    std::cerr.setf(std::ios::fixed,std::ios::floatfield);
                    std::cerr.precision(10);
                    std::cerr << "Image: dim " << ih->dim << std::endl
                              << "origin     " << ih->origin << std::endl
                              << "spacing    " << ih->spacing << std::endl
                              << "zbase      " << ih->zbase << std::endl
                              << "zscale     " << ih->zscale << std::endl;

                    Magick::Image im;
                    im.read(ih->dim[0], ih->dim[1], "R", Magick::FloatPixel, ih->pix);
                    im.colorSpace(Magick::GRAYColorspace);
                    im.quantize();
                    im.write("default.tiff");
                    std::cerr << "wrote default.tiff" << std::endl;
                    const std::string raw_name(boost::str(boost::format("default-%d.raw") % ih->dim[0]));
                    if(ih->dump_raw(raw_name))
                        std::cerr << "wrote " << raw_name << std::endl;
                    else
                        std::cerr << "couldn't write " << raw_name << std::endl;
                }
                break;
            case 'd':
                if(net)
                {
                    net->create_ramps(2.5);
                    net->remove_small_roads(50);
                    net->create_intersections();
                    net->populate_edge_hash_from_edges();

                    hwm::network hnet(hwm::from_osm("test", 0.5f, 2.5, *net));
                    hnet.build_intersections();
                    hnet.build_fictitious_lanes();
                    hnet.auto_scale_memberships();

                    try
                    {
                        hnet.check();
                        std::cerr << "HWM net checks out" << std::endl;
                    }
                    catch(std::runtime_error &e)
                    {
                        std::cerr << "HWM net doesn't check out: " << e.what() << std::endl;
                        exit(1);
                    }
                    hnet.xml_write("default.xml.gz");
                    std::cerr << "Wrote default.xml.gz" << std::endl;
                    hwm::network_aux neta(hnet);
                    neta.network_obj("default/default.obj");
                    std::cerr << "Wrote default/default.obj" << std::endl;
                }
                if(ih)
                {
                    std::cerr.setf(std::ios::fixed,std::ios::floatfield);
                    std::cerr.precision(10);
                    std::cerr << "Image: dim " << ih->dim << std::endl
                              << "origin     " << ih->origin << std::endl
                              << "spacing    " << ih->spacing << std::endl
                              << "zbase      " << ih->zbase << std::endl
                              << "zscale     " << ih->zscale << std::endl;

                    std::vector<vertex> vrts;
                    std::vector<vec3u>  fcs;
                    ih->make_mesh(vrts, fcs);
                    std::ofstream os("default.smf");
                    mesh_to_smf(os, vrts, fcs);
                        std::cerr << "Wrote default.smf" << std::endl;
                }
                break;
            case '9':
                if(ih)
                {
                    ih->zscale *= 0.5;
                    if(do_height)
                        reproject_net();
                }
                break;
            case '0':
                if(ih)
                {
                    ih->zscale *= 2;
                    if(do_height)
                        reproject_net();
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

    osm::network   *net;
    im_heightfield *ih;

    GLuint glew_state;
    GLuint tex_;
    GLuint meshlist;
    bool   do_height;
    bool   const_image;

    float       noise_scale;
    vec2f       noise_origin;
    int         octaves;
    float       power;
};

int main(int argc, char *argv[])
{
    std::cerr << libroad_package_string() << std::endl;
    if(argc != 3 && argc != 5)
    {
        std::cerr << "Usage: " << argv[0] << " <input osm or --> <input image or --> [width] [height]" << std::endl;
        return 1;
    }

    float *pix;
    vec2i  dim;
    bool   const_image;
    if(std::strcmp(argv[2], "--") == 0)
    {
        if(argc != 5)
        {
            std::cerr << "Usage: " << argv[0] << " <input osm or --> <input image or --> [width] [height]" << std::endl;

            return 1;
        }

        const_image = false;
        dim         = vec2f(atoi(argv[3]), atoi(argv[4]));
        pix         = new float[dim[0]*dim[1]];
    }
    else
    {
        Magick::Image im(argv[2]);
        im.quantizeColorSpace(Magick::GRAYColorspace);
        im.quantize();
        im.flip();

        dim = vec2i(im.columns(), im.rows());

        const_image = true;
        pix         = new float[dim[0]*dim[1]];
        im.write(0, 0, dim[1], dim[0], "R", Magick::FloatPixel, pix);
    }

    im_heightfield ih(pix, dim, 0, 2560);
    ih.origin  = vec2f(-19269.5, -18832.1);//vec2f(-50);
    ih.spacing = vec2f(75.2784);

    if(std::strcmp(argv[1], "--") == 0)
    {
        fltkview mv(0, 0, 500, 500, "fltk View");
        mv.net         = 0;
        mv.ih          = &ih;
        mv.const_image = const_image;
        mv.take_focus();
        Fl::visual(FL_DOUBLE|FL_DEPTH);

        mv.show(1, argv);
        return Fl::run();
    }
    else
    {
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

        fltkview mv(0, 0, 500, 500, "fltk View");
        mv.net         = &net;
        mv.ih          = &ih;
        mv.const_image = const_image;

        mv.take_focus();
        Fl::visual(FL_DOUBLE|FL_DEPTH);

        mv.show(1, argv);
        return Fl::run();
    }
}
