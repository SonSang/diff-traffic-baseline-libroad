#include <libroad/hwm_network.hpp>

int main(int argc, char *argv[])
{
    std::cerr << libroad_package_string() << std::endl;
    if(argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " <output file>" << std::endl;
        return 1;
    }

    hwm::network net;
    net.gamma          = 0.5;
    net.lane_width     = 2.5;
    net.name           = "qaatsi";
    const int    nx    = 3;
    const int    ny    = 17;

    const float x_len         = 60.0-net.lane_width*0.25;
    const float y_len         = 17.224-net.lane_width*0.25;
    const float y_divider_gap = 6;

    const float x_offs = y_divider_gap + 8*net.lane_width + net.lane_width*0.25;
    const float del_x  = x_len + x_offs;
    const float y_offs = 2*net.lane_width + net.lane_width*0.25;
    const float del_y  = y_len + y_offs;

    const float speedlimit = 15.0;

    for(int j = 1; j < ny-1; ++j)
    {
        for(int i = 1; i < nx-1; ++i)
        {
            hwm::intersection isec;
            isec.id = boost::str(boost::format("intersection-x%d-y%d") % i % j);
            isec.incoming.resize(10);
            isec.outgoing.resize(10);

            {
                hwm::intersection::state the_state;
                the_state.duration = 10;
                for(int s = 0; s < 8; ++s)
                    the_state.state_pairs.insert(hwm::intersection::state::state_pair(s, s));
                isec.states.push_back(the_state);
            }
            {
                hwm::intersection::state the_state;
                the_state.duration = 10;
                for(int s = 8; s < 10; ++s)
                    the_state.state_pairs.insert(hwm::intersection::state::state_pair(s, s));
                isec.states.push_back(the_state);
            }
            net.intersections.insert(std::make_pair(isec.id, isec));
        }
    }

    // make 'x' roads
    for(int j = 1; j < ny-1; ++j)
    {
        for(int i = 0; i < nx-1; ++i)
        {
            hwm::intersection_map::iterator w_isect(net.intersections.find(boost::str(boost::format("intersection-x%d-y%d") % i % j)));
            hwm::intersection_map::iterator e_isect(net.intersections.find(boost::str(boost::format("intersection-x%d-y%d") % (i+1) % j)));

            hwm::road                  new_road;

            std::vector<vec3f> pts;
            pts.push_back(vec3f( i   *del_x + x_offs/2, j*del_y, 0));
            pts.push_back(vec3f((i+1)*del_x - x_offs/2, j*del_y, 0));
            new_road.rep.initialize_from_polyline(net.lane_width, pts);

            new_road.name          = boost::str(boost::format("road-x%d%d-y%d") % i % (i+1) % j);
            new_road.id            = new_road.name;
            net.roads.insert(std::make_pair(new_road.id, new_road));
            hwm::road_map::iterator    rp(net.roads.find(new_road.id));

            {
                hwm::lane                  new_lane_we;
                new_lane_we.id         = boost::str(boost::format("lane-x%d%d-y%d-we") % i % (i+1) % j);
                hwm::lane::road_membership rm;
                rm.interval            = vec2f(0, 1);
                rm.parent_road         = &(rp->second);
                rm.lane_position       = -net.lane_width/2;
                new_lane_we.road_memberships.insert(0.0, rm);
                new_lane_we.speedlimit = speedlimit;

                net.lanes.insert(std::make_pair(new_lane_we.id, new_lane_we));
                hwm::lane_map::iterator lp(net.lanes.find(new_lane_we.id));
                if(i > 0)
                {
                    lp->second.start = new hwm::lane::intersection_terminus(&(w_isect->second), 8);
                    w_isect->second.outgoing[8] = &(lp->second);
                }
                else
                    lp->second.start = new hwm::lane::terminus();
                if(i < nx-2)
                {
                    lp->second.end   = new hwm::lane::intersection_terminus(&(e_isect->second), 8);
                    e_isect->second.incoming[8] = &(lp->second);
                }
                else
                    lp->second.end   = new hwm::lane::terminus();
            }
            {
                hwm::lane                  new_lane_ew;
                new_lane_ew.id        = boost::str(boost::format("lane-x%d%d-y%d-ew") % i % (i+1) % j);
                hwm::lane::road_membership rm;
                rm.interval           = vec2f(1, 0);
                rm.parent_road        = &(rp->second);
                rm.lane_position      = net.lane_width/2;
                new_lane_ew.road_memberships.insert(0.0, rm);
                new_lane_ew.speedlimit = speedlimit;

                net.lanes.insert(std::make_pair(new_lane_ew.id, new_lane_ew));
                hwm::lane_map::iterator lp(net.lanes.find(new_lane_ew.id));
                if(i < nx-2)
                {
                    lp->second.start            = new hwm::lane::intersection_terminus(&(e_isect->second), 9);
                    e_isect->second.outgoing[9] = &(lp->second);
                }
                else
                    lp->second.start = new hwm::lane::terminus();
                if(i > 0)
                {
                    lp->second.end              = new hwm::lane::intersection_terminus(&(w_isect->second), 9);
                    w_isect->second.incoming[9] = &(lp->second);
                }
                else
                    lp->second.end   = new hwm::lane::terminus();
            }
        }
    }

    // make 'y' roads
    for(int j = 0; j < ny-1; ++j)
    {
        for(int i = 1; i < nx-1; ++i)
        {
            hwm::intersection_map::iterator s_isect(net.intersections.find(boost::str(boost::format("intersection-x%d-y%d") % i % j)));
            hwm::intersection_map::iterator n_isect(net.intersections.find(boost::str(boost::format("intersection-x%d-y%d") % i % (j+1))));

            {
                hwm::road                  new_road;

                std::vector<vec3f> pts;
                pts.push_back(vec3f( i*del_x,  j   *del_y + y_offs/2, 0));
                pts.push_back(vec3f( i*del_x, (j+1)*del_y - y_offs/2, 0));
                new_road.rep.initialize_from_polyline(net.lane_width, pts);

                new_road.name          = boost::str(boost::format("road-x%d-y%d%d-sn") % i % j % (j+1));
                new_road.id            = new_road.name;
                net.roads.insert(std::make_pair(new_road.id, new_road));
                hwm::road_map::iterator    rp(net.roads.find(new_road.id));

                for(int l = 0; l < 4; ++l)
                {
                    hwm::lane                  new_lane_sn;
                    new_lane_sn.id         = boost::str(boost::format("lane-x%d-y%d%d-sn-%d") % i % j % (j+1) % l);
                    hwm::lane::road_membership rm;
                    rm.interval            = vec2f(0, 1);
                    rm.parent_road         = &(rp->second);
                    rm.lane_position       = -net.lane_width*(l+0.5)-y_divider_gap/2;
                    new_lane_sn.road_memberships.insert(0.0, rm);
                    new_lane_sn.speedlimit = speedlimit;
                    net.lanes.insert(std::make_pair(new_lane_sn.id, new_lane_sn));
                    hwm::lane_map::iterator    lp(net.lanes.find(new_lane_sn.id));
                    if(j > 0)
                    {
                        lp->second.start            = new hwm::lane::intersection_terminus(&(s_isect->second), l);
                        s_isect->second.outgoing[l] = &(lp->second);
                    }
                    else
                        lp->second.start   = new hwm::lane::terminus();
                    if(j < ny-2)
                    {
                        lp->second.end              = new hwm::lane::intersection_terminus(&(n_isect->second), l);
                        n_isect->second.incoming[l] = &(lp->second);
                    }
                    else
                        lp->second.end     = new hwm::lane::terminus();
                }
            }
            {
                hwm::road                  new_road;

                std::vector<vec3f> pts;
                pts.push_back(vec3f( i*del_x,  j   *del_y + y_offs/2, 0));
                pts.push_back(vec3f( i*del_x, (j+1)*del_y - y_offs/2, 0));
                new_road.rep.initialize_from_polyline(net.lane_width, pts);

                new_road.name          = boost::str(boost::format("road-x%d-y%d%d-ns") % i % j % (j+1));
                new_road.id            = new_road.name;
                net.roads.insert(std::make_pair(new_road.id, new_road));
                hwm::road_map::iterator    rp(net.roads.find(new_road.id));

                for(int l = 0; l < 4; ++l)
                {
                    hwm::lane                  new_lane_ns;
                    new_lane_ns.id         = boost::str(boost::format("lane-x%d-y%d%d-ns-%d") % i % j % (j+1) % l);
                    hwm::lane::road_membership rm;
                    rm.interval            = vec2f(1, 0);
                    rm.parent_road         = &(rp->second);
                    rm.lane_position       = net.lane_width*(l+0.5)+y_divider_gap/2;
                    new_lane_ns.road_memberships.insert(0.0, rm);
                    new_lane_ns.speedlimit = speedlimit;

                    net.lanes.insert(std::make_pair(new_lane_ns.id, new_lane_ns));
                    hwm::lane_map::iterator lp(net.lanes.find(new_lane_ns.id));
                    if(j < ny-2)
                    {
                        lp->second.start              = new hwm::lane::intersection_terminus(&(n_isect->second), l+4);
                        n_isect->second.outgoing[l+4] = &(lp->second);
                    }
                    else
                        lp->second.start = new hwm::lane::terminus();
                    if(j > 0)
                    {
                        lp->second.end                = new hwm::lane::intersection_terminus(&(s_isect->second), l+4);
                        s_isect->second.incoming[l+4] = &(lp->second);
                    }
                    else
                        lp->second.end   = new hwm::lane::terminus();
                }
            }
        }
    }

    net.build_intersections();
    net.build_fictitious_lanes();
    net.auto_scale_memberships();

    try
    {
        net.check();
        std::cerr << "HWM net checks out" << std::endl;
    }
    catch(std::runtime_error &e)
    {
        std::cerr << "HWM net doesn't check out: " << e.what() << std::endl;
        exit(1);
    }

    net.xml_write(argv[1]);

};

