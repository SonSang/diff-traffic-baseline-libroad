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

using namespace std;

static const float LANE_WIDTH = 2.5f;
static const float CAR_LENGTH = 4.5f;
//* This is the position of the car's axle from the FRONT bumper of the car
static const float CAR_REAR_AXLE = 3.5f;

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

    double accel_calc(car l, car f){
        double s1 = 2;
        double s2 = 0;
        double T = 1.6;

        //if (f.vel < 0) f.vel = 0; //TODO Should this be taken into account?

        double s_opt = s1 + T*f.vel + (f.vel*(f.vel - l.vel))/2*(sqrt(f.a_max*f.a_pref));

        double t =  f.a_max*(1 - pow((f.vel / f.v_pref),f.delta) - pow((s_opt/(l.dist - f.dist - f.length)),2));

        //cout << f.id << " accel: " << t << " vel: " << f.vel << " l:pos-diff " << l.dist - f.dist << " l vel:" << l.vel << " l id " << l.id << endl;
        // cout << f.id << " in lane " << f.lane_id  << " " << f.dist << " " << l.dist << " (" << l.id << ") "  << l.dist - f.dist << endl;

        //assert(l.dist - f.dist > 0); //A leader needs to lead.

        return t;
    }

    void calc_accel_at_isect(const lane_hash& l, car *thisCar, float _free)
    {
        hwm::lane *fict_lane = l.second.downstream_lane();

        //TODO Have the intersections update their current state
        //Test if this lane is allowed through the intersection.
        if (fict_lane)
        {
            //Find the next car ahead or the amount of free space (up to the min_for_free_mvmnt limit.
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
                    next_lane = fict_lane->downstream_lane();
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
                    if(next_lane->downstream_lane())
                    {
                        fict_lane = next_lane->downstream_lane();
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


    void calc_accel_in_isect(const lane_hash& l, car *thisCar)
    {
        hwm::lane* fict_lane = NULL;
        hwm::lane* next_lane = l.second.downstream_lane();
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
            if (next_lane->downstream_lane())
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


    void calc_lane_accel(double timestep, const pair<const str, hwm::lane> l)
    {
        //Calculate accel for all cars in lanes
        for (int i = 0; i < cars_in_lane[l.first].size(); i++)
        {
            car* thisCar = &cars_in_lane[l.first][i];
            //If car is the last in the lane,
            if (i == cars_in_lane[l.first].size() - 1)
            {
                //Behavior will be determined by the state of the intersection at the end of the lane
                if (l.second.downstream_lane())
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
    }

    void calc_all_accel(double timestep, const strhash<hwm::lane>::type& lanes, const strhash<hwm::lane>::type& i_lanes)
    {
        BOOST_FOREACH(const lane_hash& l, lanes)
        {
            calc_lane_accel(timestep, l);
        }

        //Repeat for i_lanes
        for(strhash<hwm::lane>::type::const_iterator l = i_lanes.begin(); l != i_lanes.end(); ++l)
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


    void update(double timestep, const strhash<hwm::lane>::type& lanes, const strhash<hwm::lane>::type& isect_lanes){
        typedef pair<str, hwm::lane> lane_hash;
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
        for (strhash<hwm::lane>::type::const_iterator l = isect_lanes.begin();
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
                        hwm::lane* new_lane = l.second.downstream_lane();
                        if (new_lane)
                        {
                            //Update position and distance.
                            c.dist -= c.lane_length;
                            c.lane_length = new_lane->length();
                            c.pos = (float) c.dist / c.lane_length;

                            //                        cout << "Car changed lanes: " << c.id << endl;
                            c.lane_id = new_lane->id;
                            cars_in_lane[new_lane->id].insert(cars_in_lane[new_lane->id].begin(),c);
                        }
                        else{
                            c.dist = c.lane_length;
                            c.pos = 1;
                        }
                    }
                }
            }
            cars_in_lane[l.first] = new_list_of_cars;
        }

        //Remove cars from isect_lanes if they depart and place them in lanes.
        //BOOST_FOREACH(const isect_lane
        for (strhash<hwm::lane>::type::const_iterator l = isect_lanes.begin(); l != isect_lanes.end(); l++)
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

                        hwm::lane* new_lane = l->second.downstream_lane();

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


    void new_settle(double timestep, const strhash<hwm::lane>::type& lanes, const strhash<hwm::lane>::type& i_lanes){
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


    void newer_settle(double timestep, const strhash<hwm::lane>::type& lanes, const strhash<hwm::lane>::type& i_lanes){
        double EPSILON = 1;
        double EPSILON_2 = 0.01;
        double INF = numeric_limits<double>::max();
        double max_accel = EPSILON;
        double last_max_accel = INF;
        typedef pair<str, hwm::lane> lane_hash;

        BOOST_FOREACH(const lane_hash& l, lanes)
        {
            BOOST_FOREACH(car& c, cars_in_lane[l.first])
            {
                c._last_accel = INF;
            }
        }

        do {
            max_accel = EPSILON;
            BOOST_FOREACH(const lane_hash& l, lanes)
            {
                vector<car> to_erase;
                for (deque<car>::reverse_iterator c = cars_in_lane[l.first].rbegin();
                     c != cars_in_lane[l.first].rend();
                     )
                {
                    //Try to settle a single car.  Throw it out if it cannot be settled.
                    do{

                        calc_lane_accel(timestep, l);

                        c->vel += c->accel * timestep;
                        c->vel = std::max(c->vel, 0.0);


                        if (std::abs(c->accel) > max_accel)
                        {
                            max_accel = std::abs(c->accel);
                        }

                        if ((std::abs(c->accel) > std::abs(c->_last_accel))
                            or (c->pos > 1) or ((std::abs(c->accel - c->_last_accel) < EPSILON_2) and (std::abs(c->accel) > EPSILON)))
                        {
                            deque<car>::reverse_iterator to_erase = c;
                            c++;
                            cars_in_lane[l.first].erase(to_erase.base());
                            cout << "Erasing " << to_erase->id << endl;
                            break;
                        }
                        else if (std::abs(c->accel) < EPSILON)
                        {
                            c++;
                            break;
                        }
                        else
                        {
                            c->_last_accel = c->accel;
                        }

                    } while (true);

                    if (c == cars_in_lane[l.first].rend())
                        break;
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
                network_drawer.initialize(hnet, LANE_WIDTH, 0.4f);

            setup_light();

            bb[0] = vec3f(FLT_MAX);
            bb[1] = vec3f(-FLT_MAX);
            hnet->bounding_box(bb[0], bb[1]);
        }

        glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);

        glLoadIdentity();

        glTranslatef(0.0f, 0.0f, -std::pow(2.0f, zoom));

        nav.get_rotation();
        glMultMatrixd(nav.world_mat());

        glLightfv(GL_LIGHT0, GL_POSITION, light_position.data());

        sim.update(timestep, hnet->lanes, hnet->i_lanes);

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

        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        glDisable(GL_LIGHTING);

        typedef pair<str, hwm::road> road_hash;
        BOOST_FOREACH(road_hash r, hnet->i_roads)
        {
            glColor3f(0,0,0);
            glBegin(GL_LINE_STRIP);
            BOOST_FOREACH(vec3f point, r.second.rep.points_)
            {
                glVertex3f(point[0], point[1], point[2]);
            }
            glEnd();
        }

        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        glEnable(GL_LIGHTING);

        typedef pair<str, const hwm::lane&> lane_hash;
        BOOST_FOREACH(const lane_hash &l, hnet->lanes)
        {
            BOOST_FOREACH(const car& c, sim.cars_in_lane[l.first])
            {
                mat4x4f trans(l.second.point_frame(c.pos));
                mat4x4f ttrans(tvmet::trans(trans));
                glColor3f(1.0, 0.0, 0.0);

                glPushMatrix();
                glMultMatrixf(ttrans.data());
                car_drawer.draw();
                glPopMatrix();
            }
        }


        BOOST_FOREACH(const lane_hash &l, hnet->i_lanes)
        {
            BOOST_FOREACH(const car& c, sim.cars_in_lane[l.first])
            {
                mat4x4f trans(l.second.point_frame(c.pos));
                mat4x4f ttrans(tvmet::trans(trans));
                glColor3f(1.0, 0.0, 0.0);

                glPushMatrix();
                glMultMatrixf(ttrans.data());
                car_drawer.draw();
                glPopMatrix();
            }
        }

        glDisable(GL_LIGHTING);

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

    float              car_pos;
    hwm::car_draw      car_drawer;
    hwm::network_draw  network_drawer;

    vec3f bb[2];

    int pick_vert;
    GLuint glew_state;
    vec4f light_position;
};

void timerCallback(void*)
{
    Fl::redraw();
    Fl::repeat_timeout(timestep, timerCallback);
}

int cars_per_lane = 20;
int main(int argc, char** argv)
{
    hnet = new hwm::network(hwm::load_xml_network(argv[1]));
    if(hnet->check())
        std::cerr << "HWM net checks out" << std::endl;
    else
    {
        std::cerr << "HWM net doesn't check out" << std::endl;
        exit(1);
    }
    hnet->scale_offsets(LANE_WIDTH);
    hnet->build_intersections(LANE_WIDTH);
    hnet->center();
    //    hnet->build_intersection_roads();

    //Build hash for lane lengths.
    typedef pair<str, const hwm::lane&> lane_hash;
    BOOST_FOREACH(lane_hash _lane, hnet->lanes)
    {
        lane_lengths[_lane.first] = _lane.second.length();
    }

    //TODO copy constructor for isect_lane causes segfault.
    for(strhash<hwm::lane>::type::iterator l = hnet->i_lanes.begin();
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
                p += (15.0 / _lane.second.length());
            }
        }
    }

    //Make sure car configuration is numerically stable
    sim.newer_settle(timestep, hnet->lanes, hnet->i_lanes);

    fltkview mv(0, 0, 500, 500, "fltk View");

    Fl::add_timeout(timestep, timerCallback);

    mv.take_focus();
    Fl::visual(FL_DOUBLE|FL_DEPTH);

    mv.show(1, argv);
    return Fl::run();
}
