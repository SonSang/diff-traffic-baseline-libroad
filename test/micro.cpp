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
#include <deque>
#include <algorithm>
#include <limits>


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

    bool stopping;

    double _last_accel;

    //Maybe remove: redundant data, but faster access(?)
    double lane_length;

    //Traj
    int id;

    str lane_id;

    bool operator==(const car& other) const
    {
        //Assumes we only check against cars in our lane.
        return (dist == other.dist);
    }

    car(){
        //All units are functions of meters d/or seconds
        pos = 0;
        vel = 0;
        a_max = 0.73;
        a_pref = 1.67;
        v_pref = 33;
        delta = 4;
        length = 5;
        accel = 0;
        lane_length = 0;
        dist = 0;
    }

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


//TODO Temporary (or not so temporary)
strhash<double>::type lane_lengths;

class micro : boost::noncopyable
{
public:
    //This might be better as a deque
    strhash<deque<car> >::type cars_in_lane;

    typedef pair<str, hwm::lane> lane_hash;
    typedef pair<str, hwm::isect_lane> i_lane_hash;

    double accel_calc(car l, car f){
        double s1 = 2;
        double s2 = 0;
        double T = 1.6;

        if (f.vel < 0) f.vel = 0;

        double s_opt = s1 + T*f.vel + (f.vel*(f.vel - l.vel))/2*(sqrt(f.a_max*f.a_pref));

        double t =  f.a_max*(1 - pow((f.vel / f.v_pref),f.delta) - pow((s_opt/(l.dist - f.dist - f.length)),2));



        // cout << f.id << " accel: " << t << " vel: " << f.vel << " l:pos-diff " << l.dist - f.dist << " l vel:" << l.vel << " l id " << l.id << endl;
        // cout << f.id << " in lane " << f.lane_id  << " " << f.dist << " " << l.dist << " (" << l.id << ") "  << l.dist - f.dist << endl;

        assert(l.dist - f.dist > 0); //A leader needs to lead.

        return t;
    }

    void calc_accel_at_isect(const lane_hash& l, car *thisCar, float _free)
    {
        int i_id = l.second.end.intersect_in_ref;

        //TODO Have the intersections update their current state
        //Test if this lane is allowed through the intersection.
        if (l.second.end.inters->states[l.second.end.inters->current_state].in_states[i_id].out_ref != -1)
        {
            //Find the next car ahead or the amount of free space (up to the min_for_free_mvmnt limit.
            hwm::isect_lane* fict_lane = l.second.end.inters->states[l.second.end.inters->current_state].in_states[i_id].fict_lane;
            hwm::lane* next_lane = NULL;
            double min_for_free_mvmnt = 1000;
            double free_dist = (1 - thisCar->pos) * thisCar->lane_length + _free; //Distance left in lane
            bool active = true;
            do{
                //There are no cars in the intersection lane.
                if (cars_in_lane[fict_lane->id].size() == 0)
                {
                    free_dist += fict_lane->length();
                    if (free_dist > min_for_free_mvmnt)
                    {
                        car ghost_car;
                        ghost_car.vel = 0;
                        ghost_car.dist = thisCar->dist + free_dist;

                        thisCar->accel = accel_calc(ghost_car, *thisCar);
                        active = false;
                    }
                }
                else
                {
                    //TODO move to function
                    car foo_car(cars_in_lane[fict_lane->id].front());
                    foo_car.dist += free_dist + thisCar->dist;

                    //There is a car in the lane
                    thisCar->accel = accel_calc(foo_car, *thisCar);
                    active = false;
                }
                if (active) //Check the next lane if we haven't reached a minimal free length or found a car.
                {
                    next_lane = fict_lane->output;
                    if (cars_in_lane[next_lane->id].size() == 0)
                    {
                        free_dist += next_lane->length();
                        if (free_dist > min_for_free_mvmnt)
                        {
                            car ghost_car;
                            ghost_car.vel = 0;
                            ghost_car.dist = thisCar->dist + free_dist;

                            thisCar->accel = accel_calc(ghost_car, *thisCar);

                            active = false;
                        }
                    }
                    else
                    {
                        car foo_car(cars_in_lane[next_lane->id].front());
                        foo_car.dist += free_dist + thisCar->dist;
                        thisCar->accel = accel_calc(foo_car, *thisCar);
                        active = false;
                    }
                }
                if (active) //Set up the next intersection lane if it is available, otherwise create a ghost.
                {
                    //A lane is available in the intersection.
                    if (next_lane->end.inters != NULL and next_lane->end.inters->states[next_lane->end.inters->current_state].in_states[next_lane->end.intersect_in_ref].out_ref != -1)
                    {
                        fict_lane = next_lane->end.inters->states[next_lane->end.inters->current_state].in_states[next_lane->end.intersect_in_ref].fict_lane;
                    }
                    else
                    {
                        car ghost_car;
                        ghost_car.vel = 0;
                        ghost_car.dist = thisCar->dist + free_dist;

                        thisCar->accel = accel_calc(ghost_car, *thisCar);

                        active = false;
                    }
                }
            }while(active);
        }
    }


    void calc_accel_in_isect(const i_lane_hash& l, car *thisCar)
    {
        hwm::isect_lane* fict_lane = NULL;
        hwm::lane* next_lane = l.second.output;
        assert(next_lane != NULL); //Every isect lane must end somewhere.
        double min_for_free_mvmnt = 1000;
        double free_dist = (1 - thisCar->pos) * thisCar->lane_length;
        bool active = true;

        //Check to see if next lane is empty.
        if (cars_in_lane[next_lane->id].size() == 0)
        {
            //Add next lane's length to free length.
            free_dist += next_lane->length();
            //Do not recurse if we've reached an amount of free space such that additional quantities would yield diminishing returns.
            if (free_dist > min_for_free_mvmnt)
            {
                //TODO move these calculations to a function
                car ghost_car;
                ghost_car.vel = 0;
                //Locate ghost car such that there is "free_dist" between it and the current car.
                ghost_car.dist = thisCar->dist + free_dist;

                thisCar->accel = accel_calc(ghost_car, *thisCar);

                active = false;
            }
        }
        else //There are cars in the lane, so use car in the front of the lane (confusingly, the last car.) last car
        {
            //TODO should be in a function
            car foo_car(cars_in_lane[next_lane->id].front());
            foo_car.dist += free_dist + thisCar->dist;

            thisCar->accel = accel_calc(foo_car, *thisCar);
            active = false;
        }
        if(active) //Only called if we need to check further lanes.
        {
            //A lane is available in the intersection.
            if (next_lane->end.inters != NULL and next_lane->end.inters->states[next_lane->end.inters->current_state].in_states[next_lane->end.intersect_in_ref].out_ref != -1)
            {
                //Call to calc_accel_at_isect
                calc_accel_at_isect(make_pair(next_lane->id, *next_lane), thisCar, free_dist);
            }
            else //The next lane does not lead anywhere or has a red light.
            {
                car ghost_car;
                ghost_car.vel = 0;
                ghost_car.dist = thisCar->dist + free_dist;

                thisCar->accel = accel_calc(ghost_car, *thisCar);

                active = false;
            }
        }
    }



    void calc_all_accel(double timestep, const strhash<hwm::lane>::type& lanes, const strhash<hwm::isect_lane>::type& i_lanes)
    {
        BOOST_FOREACH(const lane_hash& l, lanes)
        //Calculate accel for all cars in lanes
        for (int i = 0; i < cars_in_lane[l.first].size(); i++)
        {
            car* thisCar = &cars_in_lane[l.first][i];
            //If car is the last in the lane,
            if (i == cars_in_lane[l.first].size() - 1)
            {
                //Behavior will be determined by the state of the intersection at the end of the lane
                if ((l.second.end.inters != NULL) and (l.second.end.inters->states[l.second.end.inters->current_state].in_states[l.second.end.intersect_in_ref].out_ref != -1))
                {
                    calc_accel_at_isect(l, thisCar, 0);
                    thisCar->stopping = false;
                }
                else //If the lane ends, there is a red light, or the car wants to go a different direction.
                {
                    car ghost_car(1, 0, cars_in_lane[l.first][i].lane_length);

                    thisCar->stopping = true;
                    cars_in_lane[l.first][i].accel = accel_calc(ghost_car, cars_in_lane[l.first][i]);
                }
            }
            else //Otherwise, use the car ahead.
            {
                thisCar->stopping = false;
                cars_in_lane[l.first][i].accel = accel_calc(cars_in_lane[l.first][i + 1], cars_in_lane[l.first][i]);
            }
        }

        //Repeat for i_lanes
        for(strhash<hwm::isect_lane>::type::const_iterator l = i_lanes.begin(); l != i_lanes.end(); ++l)
        {
            //Calculate accel for all cars in lanes
            for (int i = 0; i < cars_in_lane[l->first].size(); i++)
            {
                car* thisCar = &cars_in_lane[l->first][i];
                //If the car is the last in the lane,
                if (i == cars_in_lane[l->first].size() - 1)
                {
                    calc_accel_in_isect(make_pair(l->first, l->second), thisCar);
                    thisCar->stopping = false;
                }
                else //Otherwise use the car ahead.
                {
                    thisCar->stopping = false;
                    cars_in_lane[l->first][i].accel = accel_calc(cars_in_lane[l->first][i + 1], cars_in_lane[l->first][i]);
                }
            }
        }
    }


    void update(double timestep, const strhash<hwm::lane>::type& lanes, const strhash<hwm::isect_lane>::type& isect_lanes){
        typedef pair<str, hwm::lane> lane_hash;
        typedef pair<str, hwm::isect_lane> isect_lane_hash;
        typedef pair<str, hwm::intersection> isect_hash;

        calc_all_accel(timestep, lanes, isect_lanes);

        map<str, deque<car> > new_cars_in_lane;
        //Update all cars in lanes
        BOOST_FOREACH(const lane_hash& l, lanes)
        {
            BOOST_FOREACH(car& c, cars_in_lane[l.first])
            {

                c.dist += c.vel * timestep;
                c.vel += c.accel * timestep;
                c.pos = c.dist / c.lane_length; //TODO this does not need to be calculated here, if it is inefficient
            }

        }

        //Update all cars in isect_lanes

        //        BOOST_FOREACH(const isect_lane_hash& l, isect_lanes) TODO Copy constructor causes segfault when lane adjacency is copied
        for (strhash<hwm::isect_lane>::type::const_iterator l = isect_lanes.begin();
             l != isect_lanes.end();
             l++)
        {
            BOOST_FOREACH(car& c, cars_in_lane[l->first])
            {
                c.dist += c.vel * timestep;
                c.vel += c.accel * timestep;
                c.pos = c.dist / c.lane_length; //TODO this does not need to be calculated here, if it is inefficient
            }
        }

        //Remove cars from lanes if they depart and place them in isect_lanes.
        BOOST_FOREACH(const lane_hash& l, lanes)
        {
            deque<car> new_list_of_cars;
            BOOST_FOREACH(car& c, cars_in_lane[l.first])
            {
                if (c.dist < c.lane_length)
                {
                    new_list_of_cars.push_back(c);
                }
                else
                {
                    //ASSUMES car will not clear next lane in a timestep
                    //Remove car from lane
                    //->This is handled implicitly
                    if (not isnan(c.dist) and  not isinf(c.dist))
                    {

                        hwm::lane* new_lane = l.second.end.inters->states[l.second.end.inters->current_state].in_states[l.second.end.intersect_in_ref].fict_lane;

                        //Update position and distance.
                        c.dist -= c.lane_length;
                        c.lane_length = new_lane->length();
                        c.pos = (float) c.dist / c.lane_length;


                        //                        cout << "Car changed lanes: " << c.id << endl;
                        c.lane_id = new_lane->id;
                        cars_in_lane[new_lane->id].insert(cars_in_lane[new_lane->id].begin(),c);
                    }
                }
            }
            cars_in_lane[l.first] = new_list_of_cars;
        }

        //Remove cars from isect_lanes if they depart and place them in lanes.
        //BOOST_FOREACH(const isect_lane
        for (strhash<hwm::isect_lane>::type::const_iterator l = isect_lanes.begin(); l != isect_lanes.end(); l++)
        {
            deque<car> new_list_of_cars;
            BOOST_FOREACH(car& c, cars_in_lane[l->first])
            {
                if (c.dist < c.lane_length)
                {
                    new_list_of_cars.push_back(c);
                }
                else
                {

                    if (not isnan(c.dist) and  not isinf(c.dist))
                    {
                        //ASSUMES car will not clear next lane in a timestep
                        //Remove car from lane
                        //->This is handled implicitly

                        hwm::lane* new_lane = l->second.output;

                        //Update position and distance.
                        c.dist -= c.lane_length;
                        c.lane_length = new_lane->length();
                        c.pos = (float) c.dist / c.lane_length;

                        //                        cout << "Car changed lanes: " << c.id << endl;
                        c.lane_id = new_lane->id;
                        cars_in_lane[new_lane->id].insert(cars_in_lane[new_lane->id].begin(),c);
                    }
                }
            }
            cars_in_lane[l->first] = new_list_of_cars;
        }

    }


    void new_settle(double timestep, const strhash<hwm::lane>::type& lanes, const strhash<hwm::isect_lane>::type& i_lanes){
        double EPSILON = 1;
        double INF = numeric_limits<double>::max();
        double max_accel = EPSILON;
        double last_max_accel = INF;
        typedef pair<str, hwm::lane> lane_hash;
        do {

            max_accel = EPSILON;

            calc_all_accel(timestep, lanes, i_lanes);

            BOOST_FOREACH(const lane_hash& l, lanes)
            {
                BOOST_FOREACH(car& c, cars_in_lane[l.first])
                {
                    c._last_accel = INF;
                }
            }

            BOOST_FOREACH(const lane_hash& l, lanes)
            {
                vector<car> to_erase;
                BOOST_FOREACH(car& c, cars_in_lane[l.first])
                {
                    c.vel += c.accel * timestep;

                    if (std::abs(c.accel) > max_accel)
                    {
                        max_accel = std::abs(c.accel);
                    }

                    if (std::abs(c.accel) > std::abs(c._last_accel))
                    {
                        to_erase.push_back(c);
                    }

                    c._last_accel = c.accel;
                }

                BOOST_FOREACH(car& c, to_erase)
                {
                    cars_in_lane[l.first].erase(find(cars_in_lane[l.first].begin(), cars_in_lane[l.first].end(), c));
                    cout << "Car removed " << c.id << endl;
                }
            }

            // if (max_accel >= last_max_accel)
            // {
            //     cout << "The cars did not settle." << endl;
            //     exit(0);
            // }
            last_max_accel = max_accel;
            cout << "max: " << max_accel << endl;
        } while (max_accel > EPSILON);

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
                        cars_in_lane[l.first][i].vel += cars_in_lane[l.first][i].accel * timestep;
                        cars_in_lane[l.first][i].vel = max(cars_in_lane[l.first][i].vel, 0.0);
                        if (abs(cars_in_lane[l.first][i].accel) > max_accel)
                            max_accel = abs(cars_in_lane[l.first][i].accel);



                    }
                    else
                    {

                        cars_in_lane[l.first][i].accel = accel_calc(cars_in_lane[l.first][i + 1], cars_in_lane[l.first][i]);

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

    sim.update(timestep, hnet->lanes, hnet->i_lanes);

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

            glBegin(GL_POLYGON);
            glVertex3f(0,1,0);
            glVertex3f(0,-1,0);
            glVertex3f(5,-1,0);
            glVertex3f(5,1,0);
            glEnd();
        }
    }


    for (strhash<hwm::isect_lane>::type::iterator l = hnet->i_lanes.begin(); l != hnet->i_lanes.end(); l++)
    {
        glLoadIdentity();

        BOOST_FOREACH(const car& c, sim.cars_in_lane[l->first]){
            glLoadIdentity();

            vec3f car_pos = l->second.point(c.pos);
            glTranslatef(car_pos[0], car_pos[1], car_pos[2]);

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

    //Build hash for lane lengths.
    typedef pair<str, const hwm::lane&> lane_hash;
    BOOST_FOREACH(lane_hash _lane, hnet->lanes)
    {
        lane_lengths[_lane.first] = _lane.second.length();
    }

    //TODO copy constructor for isect_lane causes segfault.
    for(strhash<hwm::isect_lane>::type::iterator l = hnet->i_lanes.begin();
        l != hnet->i_lanes.end();
        l++)
    {
        lane_lengths[l->first] = l->second.length();
    }

    //Create some sample cars
    BOOST_FOREACH(lane_hash _lane, hnet->lanes)
    {
        if (_lane.first == str("lane0c") or true)
        {
            double p = 0.1;
            for (int i = 0; i < cars_per_lane; i++)
            {
                //TODO Just creating some cars here...
                car tmp(p, 33, lane_lengths[_lane.first]);
                tmp.lane_id = _lane.first;
                tmp.id = rand();
                sim.cars_in_lane[_lane.second.id].push_back(tmp);
                //Cars need a minimal distance spacing
                p += 0.4;
            }
        }
    }

    //Make sure car configuration is numerically stable
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
