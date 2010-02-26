#include <libroad/osm_network.hpp>
#include <libroad/hwm_network.hpp>
#include <iostream>
#include <FL/Fl.H>
#include <FL/Fl_Window.H>
#include <FL/Fl_Box.H>
#include <FL/gl.h>
#include <FL/glu.h>
#include <FL/glut.H>
#include <FL/Fl_Gl_Window.H>
#include <FL/Fl_Dial.H>
#include <FL/Fl_Button.H>
#include <FL/Fl_Double_Window.H>
#include <FL/Fl_Float_Input.H>

#include <boost/foreach.hpp>

class glWindow : public Fl_Gl_Window {
    void draw();
    int handle(int);

public:
    glWindow(int X, int Y, int W, int H, const char *L) : Fl_Gl_Window(X, Y, W, H, L){}
};

int glWindow::handle(int event){ return 0;}

osm::network* net;

void glWindow::draw(){
    if (!valid()) {
        valid(1);
        glViewport(0,0,w(),h());
        glMatrixMode (GL_PROJECTION);
        glLoadIdentity ();
        gluPerspective(60.0, (GLdouble) w()/(GLdouble) h(), 0.0, -1.0);
        gluLookAt(0,0,4000,0,0,0,0,1,0);
        glMatrixMode (GL_MODELVIEW);
        glLoadIdentity();
        glEnable (GL_BLEND);
        glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    }
    glClearColor(1, 1, 1, 1);
    glClear(GL_COLOR_BUFFER_BIT);

    glLoadIdentity();

    net->draw_network();

    glLoadIdentity();
}

double timestep = 0.2;

void timerCallback(void*)
{
    Fl::redraw();
    Fl::repeat_timeout(timestep, timerCallback);
}

int main(int argc, char *argv[])
{
    //Create from grid
    // osm::network s_net;
    // net = &s_net;
    // net->create_grid(6, 6, 200, 200);
    // net->compute_edge_types();
    // net->compute_node_degrees();
    // net->join_logical_roads();
    // net->split_into_road_segments();
    // net->remove_small_roads(15);
    // net->create_intersections();
    // net->populate_edge_hash_from_edges();
    // hwm::network hnet(hwm::from_osm("test", 0.5f, *net));
    // write_xml_network(hnet, "test_net.xml");

    //Load from file
    osm::network s_net(osm::load_xml_network(argv[1]));
    net = &s_net;
    net->populate_edges_from_hash();
    net->clip_roads_to_bounds();
    net->compute_edge_types();
    net->compute_node_degrees();
    net->scale_and_translate();
    net->split_into_road_segments();
    net->remove_highway_intersections();
    net->compute_node_heights();
    net->join_logical_roads();
    net->join_logical_roads();
    net->create_ramps();
    // net->join_logical_roads();
    // net->join_logical_roads();
    net->remove_small_roads(40);
    net->create_intersections();
    net->populate_edge_hash_from_edges();
    hwm::network hnet(hwm::from_osm("test", 0.5f, 2.5, *net));
    hnet.xml_write("test_net.xml");

    //if (hnet.check())
    //    std::cerr << "Conversion seems to have worked\n";

    Fl_Double_Window *window = new Fl_Double_Window(500,500);
    Fl::add_timeout(timestep, timerCallback);
    glWindow* glWin = new glWindow(0, 0, window->w(), window->h(), 0);
    glWin->mode(FL_DOUBLE);

    window->end();
    window->show(argc, argv);
    glWin->show();

    Fl::redraw();

    return Fl::run();
}
