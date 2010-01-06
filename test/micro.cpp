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
#include <unistd.h>


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

    double _last_accel;

    //Maybe remove: redundant data, but faster access(?)
    double lane_length;

    //Traj
    str id;

    car(double p, double v, double _lane_length){
        //All units are functions of meters d/or seconds
        pos = p;
        vel = v;
        a_max = 0.73;
        a_pref = 1.67;
        v_pref = 33;
        delta = 4;
        length = 5;
        accel = 0;
        lane_length = _lane_length;
        dist = p * _lane_length;
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
            cout << " inters " << l.second.end.inters << endl;
            if (l.second.end.inters != NULL)
            {
                int i_id = l.second.end.intersect_in_ref;

                if (l.second.end.inters->states[0].in_states[i_id].out_ref != -1)
                {
                    cout << "in ref is " << i_id << endl;
                    cout << "next lane id " << l.second.end.inters->states[0].in_states[i_id].out_ref << endl;
                    cout << "next lane name " << l.second.end.inters->outgoing[l.second.end.inters->states[0].in_states[i_id].out_ref]->id << endl;
                }
            }

            for (int i = 0; i < cars_in_lane[l.first].size(); i++)
            {
                if (i == cars_in_lane[l.first].size() - 1)
                {
                    //Behavior will be determined by the state of the intersection at the end of the lane
                    //ASSUMES car is moving positively
                    car ghost_car(1, 0, cars_in_lane[l.first][i].lane_length);

                    cars_in_lane[l.first][i].accel = accel_calc(ghost_car, cars_in_lane[l.first][i]);
                }
                else
                {
                    cars_in_lane[l.first][i].accel = accel_calc(cars_in_lane[l.first][i + 1], cars_in_lane[l.first][i]);
                }
            }
        }

        BOOST_FOREACH(const lane_hash& l, lanes)
        {
            BOOST_FOREACH(car& c, cars_in_lane[l.first])
            {
                c.dist += c.vel * timestep;
                c.vel += c.accel * timestep;
                c.pos = c.dist / c.lane_length; //TODO this does not need to be calculated here, if it is inefficient
            }
        }
    }


    void settle(double timestep, const strhash<hwm::lane>::type& lanes){
        double EPSILON = 0.1;
        double max_accel = EPSILON;
        double last_max_accel = 9999999;
        typedef pair<str, hwm::lane> lane_hash;
        do {
            max_accel = EPSILON;
            BOOST_FOREACH(const lane_hash& l, lanes)
            {

                for (int i = 0; i < cars_in_lane[l.first].size(); i++)
                {
                    if (i == cars_in_lane[l.first].size() - 1)
                    {
                        //Behavior will be determined by the state of the intersection at the end of the lane
                        //ASSUMES car is moving positively
                        car ghost_car(1, 0, cars_in_lane[l.first][i].lane_length);

                        cars_in_lane[l.first][i].accel = accel_calc(ghost_car, cars_in_lane[l.first][i]);
//                         cout << l.first << " " << i << " accel:" << cars_in_lane[l.first][i].accel << " dist:" << cars_in_lane[l.first][i].pos*cars_in_lane[l.first][i].lane_length << " vel:" << cars_in_lane[l.first][i].vel <<  " total:"
//                              << cars_in_lane[l.first][i].lane_length << endl;


                        cars_in_lane[l.first][i].vel += cars_in_lane[l.first][i].accel * timestep;
                        cars_in_lane[l.first][i].vel = max(cars_in_lane[l.first][i].vel, 0.0);
                        if (abs(cars_in_lane[l.first][i].accel) > max_accel)
                            max_accel = abs(cars_in_lane[l.first][i].accel);



                    }
                    else
                    {

                        cars_in_lane[l.first][i].accel = accel_calc(cars_in_lane[l.first][i + 1], cars_in_lane[l.first][i]);
//                         cout << l.first << " " << i << " accel:" << cars_in_lane[l.first][i].accel << " dist:" << cars_in_lane[l.first][i].pos*cars_in_lane[l.first][i].lane_length << " vel:" << cars_in_lane[l.first][i].vel <<  " total:"
//                              << cars_in_lane[l.first][i].lane_length << endl;

                        cars_in_lane[l.first][i].vel += cars_in_lane[l.first][i].accel * timestep;
                        cars_in_lane[l.first][i].vel = max(cars_in_lane[l.first][i].vel, 0.0);
                        if (abs(cars_in_lane[l.first][i].accel) > max_accel)
                            max_accel = abs(cars_in_lane[l.first][i].accel);


                    }
                }
            }

            if (max_accel >= last_max_accel)
            {
                cout << "The cars did not settle." << endl;
                exit(0);
            }
            last_max_accel = max_accel;
        } while (max_accel > EPSILON);
    }
};


/******************
Globals for OpenGL Loop
 */
double timestep = 0.033;
hwm::network* hnet;
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
        gluLookAt(0,0,200,0,0,0,0,1,0);
        glMatrixMode (GL_MODELVIEW);
        glLoadIdentity();
        glEnable (GL_BLEND); glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    }

    glClearColor(1, 1, 1, 1);
    glClear(GL_COLOR_BUFFER_BIT);

    sim.update(timestep, hnet->lanes);

    typedef pair<str, hwm::road> road_hash;
    BOOST_FOREACH(road_hash r, hnet->roads)
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


    BOOST_FOREACH(road_hash r, hnet->i_roads)
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

    glColor3f(1,0,0);
    typedef pair<str, const hwm::lane&> lane_hash;
    BOOST_FOREACH(lane_hash l, hnet->lanes)
    {
        glLoadIdentity();

        BOOST_FOREACH(const car& c, sim.cars_in_lane[l.first]){
            glLoadIdentity();

            vec3f car_pos = l.second.point(c.pos);
            glTranslatef(car_pos[0], car_pos[1], car_pos[2]);

            // cout << "Car at " << car_pos[0]
            //      << " " << car_pos[1]
            //      << " " << car_pos[2] << endl;
            // cout << "Its p is " << c.pos << endl;

            glBegin(GL_POLYGON);
            glVertex3f(0,1,0);
            glVertex3f(0,-1,0);
            glVertex3f(5,-1,0);
            glVertex3f(5,1,0);
            glEnd();
        }
    }
}


void timerCallback(void*)
{
    Fl::redraw();
    Fl::repeat_timeout(timestep, timerCallback);
}


void lane_test(const hwm::network&);

int cars_per_lane = 2;
int main(int argc, char** argv)
{
    hnet = new hwm::network(hwm::load_xml_network(argv[1]));
    hnet->build_intersection_roads();

    typedef pair<str, const hwm::lane&> lane_hash;

    BOOST_FOREACH(lane_hash _lane, hnet->lanes)
    {
        lane_lengths[_lane.first] = _lane.second.length();
    }

    BOOST_FOREACH(lane_hash _lane, hnet->lanes)
    {
            double p = 0.1;
            for (int i = 0; i < cars_per_lane; i++)
            {
                //TODO Just creating some cars here...
                sim.cars_in_lane[_lane.second.id].push_back(car(p, 33, lane_lengths[_lane.first]));
                //Cars need a minimal distance spacing
                p += 0.4;
            }
    }

    sim.settle(timestep, hnet->lanes);

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


void lane_test(const hwm::network& net)
{
    typedef pair<str, hwm::lane> lane_hash;
    BOOST_FOREACH(const lane_hash& p, net.lanes)
    {
        if (p.second.road_memberships.size() > 0)
            cout << p.second.point(0.5) << " is 0.5, -.5 is " << p.second.point(-0.5) << endl;
    }
}
