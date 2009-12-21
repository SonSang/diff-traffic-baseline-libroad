#include "micro.hpp"
#include <libroad/hwm_network.hpp>

#include <utility>
#include <iostream>
#include <cmath>

#include <boost/foreach.hpp>
#include <boost/utility.hpp>

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

using namespace std;

class car
{
public:
    double pos;
    double vel;
    double a_max;
    double a_pref;
    double v_pref;
    double delta;
    double length;
    double accel;
    double dist;

    //Traj
    str id;

    car(double p, double v, double scale){
        //All units are functions of meters d/or seconds
        pos = p;
        vel = v;
        a_max = 0.73;
        a_pref = 1.67;
        v_pref = 33;
        delta = 4;
        length = 5;
        accel = 0;
        dist = p * scale;
    }
};


//Temporary
strhash<double>::type lane_lengths;

class micro : boost::noncopyable
{
public:
    //This might be better as a deque
    strhash<vector<car> >::type cars_in_lane;

    double accel_calc(car l, car f){
        double s1 = 2;
        double s2 = 0;
        double T = 1.6;
        double s_opt = s1 + T*f.vel + (f.vel*(f.vel - l.vel))/2*(sqrt(f.a_max*f.a_pref));
        return f.a_max*(1 - pow((f.vel / f.v_pref),f.delta) - pow((s_opt/(l.dist - f.dist - f.length)),2));
    }

    void update(double timestep, const strhash<hwm::lane>::type& lanes){
        typedef pair<str, hwm::lane> lane_hash;
        BOOST_FOREACH(const lane_hash& l, lanes)
        {
            for (int i = 0; i < cars_in_lane[l.first].size(); i++)
            {
                if (i == 0)
                {
                    //Behavior will be determined by the state of the intersection at the end of the lane
                }
                else
                {
                    cars_in_lane[l.first][i].accel = accel_calc(cars_in_lane[l.first][i - 1], cars_in_lane[l.first][i]);
                }
            }
        }

        BOOST_FOREACH(const lane_hash& l, lanes)
        {
            BOOST_FOREACH(car& c, cars_in_lane[l.first])
            {
                c.dist += c.vel * timestep;
                c.vel += c.accel * timestep;
                c.pos = c.pos; //TODO The _INTERVAL_ position must be recalculated.  Lane memebership should be updated based an this.
            }
        }
    }
};


/******************
Globals for OpenGL Loop
 */
double timestep = 0.033;
hwm::network hnet;
micro sim;


class glWindow : public Fl_Gl_Window {
    void draw();
    int handle(int);

public:
    glWindow(int X, int Y, int W, int H, const char *L) : Fl_Gl_Window(X, Y, W, H, L){}
};

int glWindow::handle(int event){ return 0;}

void glWindow::draw(){
    if (!valid()) {
        valid(1);
        glViewport(0,0,w(),h());
        glMatrixMode (GL_PROJECTION);
        glLoadIdentity ();
        gluPerspective(60.0, (GLdouble) w()/(GLdouble) h(), 1.0, -1.0);
        gluLookAt(0,0,600,0,0,0,0,1,0);
        glMatrixMode (GL_MODELVIEW);
        glLoadIdentity();
        glEnable (GL_BLEND); glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    }

    glClearColor(1, 1, 1, 1);
    glClear(GL_COLOR_BUFFER_BIT);

    sim.update(timestep, hnet.lanes);

    typedef pair<str, hwm::road> road_hash;
    BOOST_FOREACH(road_hash r, hnet.roads)
    {
        glLoadIdentity();
        glColor3f(0,0,0);
        glBegin(GL_LINE_STRIP);
        BOOST_FOREACH(vec3f point, r.second.rep.points_)
        {
            glVertex3f(point[0], point[1], point[2]);
        }
        glEnd();
    }
}


void timerCallback(void*)
{
    Fl::redraw();
    Fl::repeat_timeout(timestep, timerCallback);
}

//TODO TEMP need an actual computation of physical length from interval position
double intr_to_len = 300;

//
int cars_per_lane = 6;
int main(int argc, char** argv)
{
    hnet = hwm::load_xml_network(argv[1]);
    write_xml_network(hnet, "test.xml");
    //Can't export the xml network..


    //***********************
    // List of errors encountered and changes required. (Priority)
    // 1, Error: there is a lane with a blank id and no road membership. (Low)
    // 2, Error: the road representation data is not correct in the road memberships.

    typedef pair<str, const hwm::lane&> lane_hash;
    BOOST_FOREACH(lane_hash _lane, hnet.lanes)
    {
        //TODO Why are there lanes being read with no read memberships and no id?
        //Error 1
        if (_lane.second.road_memberships.size() > 0)
        {
            double p = 0.01;
            for (int i = 0; i < cars_per_lane; i++)
            {
                //TODO Just creating some cars here...
                sim.cars_in_lane[_lane.second.id].push_back(car(p, 33, intr_to_len));
                p += 0.04;
            }
        }
    }

    //Temproray: approximately calculate total length of lanes
    //Error 2: Why isn't the representation data correct?  Causes SEGFAULTS.
    BOOST_FOREACH(lane_hash _lane, hnet.lanes)
    {
        lane_lengths[_lane.first] = _lane.second.length();
    }

    Fl_Double_Window *window = new Fl_Double_Window(500,500);
    Fl::add_timeout(timestep, timerCallback);
    glWindow* glWin = new glWindow(0, 0, window->w(), window->h(), 0);
    glWin->mode(FL_DOUBLE);

    window->end();
    window->show(argc, argv);
    glWin->show();

    return Fl::run();

    return 0;
}
