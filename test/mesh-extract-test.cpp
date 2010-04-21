#include "libroad/hwm_network.hpp"
#include "texture-gen.hpp"
#include "boost/filesystem.hpp"

namespace bf = boost::filesystem;

void dump_obj(std::ostream      &out,
              const std::string &name,
              const std::string &material_name,
              const std::vector<vertex> &verts,
              const std::vector<vec3u>  &faces)
{
    out << "o " << name << "\n";
    out << "usemtl " << material_name << "\n";
    BOOST_FOREACH(const vertex &v, verts)
    {
        out << "v " << v.position[0] << " " << v.position[1] << " " << v.position[2]  << "\n"
            << "vn " << v.normal[0] << " " << v.normal[1] << " " << v.normal[2]  << "\n"
            << "vt " << v.tex_coord[0] << " " << v.tex_coord[1] << "\n";
    }

    const size_t nverts = verts.size();
    BOOST_FOREACH(const vec3u &f, faces)
    {
        out << "f ";
        BOOST_FOREACH(const unsigned int i, f)
        {
            const off_t idx = static_cast<off_t>(i) - nverts;
            out << idx << "/" << idx << "/" << idx << " ";
        }
        out << "\n";
    }
};

void write_mtl(std::ostream &o,
               const std::string &ts_name,
               const std::string &ts_file)
{
    o << boost::str(boost::format("newmtl %s\n"
                                  "ns 96.078431\n"
                                  "ka 0.0  0.0  0.0\n"
                                  "kd 0.64 0.64 0.64\n"
                                  "ks 0.5  0.5  0.5\n"
                                  "ni 1.0\n"
                                  "d  1.0\n"
                                  "map_kd %s\n") % ts_name % ts_file);
}

struct road_rev_map
{
    struct lane_entry
    {
        lane_entry()
        {}

        lane_entry(const hwm::lane* l, const hwm::lane::road_membership *rm) :
            lane(l), membership(rm)
        {}

        const hwm::lane                  *lane;
        const hwm::lane::road_membership *membership;
    };

    struct lane_cont : public std::map<const float, const lane_entry>
    {
        void write_texture(const std::string &texfile) const
        {
            lane_maker lm;
            // lm.add_xgap(0.25*lane_width);
            // lm.add_cbox(new single_box(line_width, line_length+line_gap_length,
            //                            0,          line_length+line_gap_length,
            //                            color4d(1.0, 1.0, 1.0, 1.0)));

            const_iterator i = begin();
            const float orient0 = copysignf(1, i->second.membership->interval[0] - i->second.membership->interval[1]);
            while(i != end() && orient0 == copysignf(1, i->second.membership->interval[0] - i->second.membership->interval[1]))
            {
                lm.add_xgap(lane_width);
                const_iterator next = boost::next(i);
                if(next != end() && orient0 == copysignf(1, next->second.membership->interval[0] - next->second.membership->interval[1]))
                    lm.add_cbox(new single_box(line_width, line_length+line_gap_length,
                                               0, line_length,
                                               color4d(1.0, 1.0, 1.0, 1.0)));
                i = next;
            }
            if(i != end() && !lm.boxes.empty())
            {
                lm.add_cbox(new double_box(line_width, line_sep_width,
                                           line_length+line_gap_length, 0, line_length+line_gap_length,
                                           color4d(1.0, 1.0, 0.0, 1.0)));
            }
            while(i != end())
            {
                lm.add_xgap(lane_width);
                const_iterator next = boost::next(i);
                if(next != end())
                    lm.add_cbox(new single_box(line_width, line_length+line_gap_length,
                                               0, line_length,
                                               color4d(1.0, 1.0, 1.0, 1.0)));
                i = next;
            }
            // lm.add_cbox(new single_box(line_width, line_length+line_gap_length,
            //                            0,          line_length+line_gap_length,
            //                            color4d(1.0, 1.0, 1.0, 1.0)));
            // lm.add_xgap(0.25*lane_width);

            lm.draw(texfile);
        }

        void make_mesh(std::vector<vertex> &vrts, std::vector<vec3u> &fcs, const vec2f &interval) const
        {
            const float left  = begin()            ->first-0.5f*lane_width/*-0.25f*lane_width*/;
            const float right = boost::prior(end())->first+0.5f*lane_width/*+0.25f*lane_width*/;
            const hwm::lane::road_membership &rm = *(begin()->second.membership);
            rm.parent_road->rep.make_mesh(vrts, fcs, interval, vec2f(left, right), 0.01, true);
        }
    };

    partition01<lane_cont>  lane_map;
    const hwm::road        *road;

    road_rev_map()
    {
        lane_map.insert(0.0f, lane_cont());
    }

    road_rev_map(const hwm::road *r) : road(r)
    {
        lane_map.insert(0.0f, lane_cont());
    }

    void add_lane(const hwm::lane *r, const hwm::lane::road_membership *rm)
    {
        vec2f interval(rm->interval);
        lane_entry le(r, rm);
        if(interval[0] > interval[1])
            std::swap(interval[0], interval[1]);

        partition01<lane_cont>::iterator start(lane_map.find(interval[0]));
        const vec2f         start_interval(lane_map.containing_interval(start));

        start = lane_map.split_interval(start, vec2f(interval[0], std::min(interval[1], start_interval[1])), start->second);

        start->second.insert(std::make_pair(le.membership->lane_position, le));
        if(interval[1] <= start_interval[1])
            return;

        partition01<lane_cont>::iterator end(lane_map.find(interval[1]));
        const vec2f         end_interval(lane_map.containing_interval(end));

        end = lane_map.split_interval(end, vec2f(end_interval[0], interval[1]), end->second);
        end->second.insert(std::make_pair(le.membership->lane_position, le));

        for(partition01<lane_cont>::iterator current = boost::next(start); current != end; ++current)
            current->second.insert(std::make_pair(le.membership->lane_position, le));
    }

    void print() const
    {
        for(partition01<lane_cont>::const_iterator current = lane_map.begin(); current != lane_map.end(); ++current)
        {
            std::cout << lane_map.containing_interval(current) << ": ";
            BOOST_FOREACH(const lane_cont::value_type &le, current->second)
            {
                std::cout << le.second.membership->lane_position << " " << le.second.lane->id;
                if(le.second.membership->interval[0] < le.second.membership->interval[1])
                    std::cout << "(+) ";
                else
                    std::cout << "(-) ";

            }
            std::cout << std::endl;
        }
    }
};

void obj_roads(std::ostream &os, hwm::network &net)
{
    strhash<road_rev_map>::type rrm;

    BOOST_FOREACH(const hwm::road_pair &r, net.roads)
    {
        rrm[r.first] = road_rev_map(&(r.second));
    }

    BOOST_FOREACH(const hwm::lane_pair &l, net.lanes)
    {
        BOOST_FOREACH(const hwm::lane::road_membership::intervals::entry &rm, l.second.road_memberships)
        {
            strhash<road_rev_map>::type::iterator rev_itr(rrm.find(rm.second.parent_road->id));
            assert(rev_itr != rrm.end());
            rev_itr->second.add_lane(&(l.second), &(rm.second));
        }
    }

    const std::string mtlname("road.mtl");
    std::ofstream mtllib(mtlname.c_str());

    os << "mtllib " << mtlname << std::endl;
    BOOST_FOREACH(const strhash<road_rev_map>::type::value_type &rrm_v, rrm)
    {
        const hwm::road &r = *(rrm_v.second.road);

        size_t re_c = 0;
        for(partition01<road_rev_map::lane_cont>::const_iterator current = rrm_v.second.lane_map.begin();
            current != rrm_v.second.lane_map.end();
            ++current)
        {
            const road_rev_map::lane_cont &e = current->second;

            if(e.empty())
                continue;

            std::vector<vertex> vrts;
            std::vector<vec3u>  fcs;
            e.make_mesh(vrts, fcs, rrm_v.second.lane_map.containing_interval(current));

            const std::string &oname(boost::str(boost::format("%s-%d") % r.id % re_c));
            const std::string &texfilename(boost::str(boost::format("tex/%s.png") % oname));
            e.write_texture(texfilename);
            dump_obj(os,
                     oname,
                     oname,
                     vrts,
                     fcs);

            write_mtl(mtllib, oname, texfilename);
            ++re_c;
        }
    }
}

struct oriented_membership
{
    oriented_membership(bool i, const hwm::lane *l, const hwm::lane::road_membership *m)
        : incomingp(i), lane(l), membership(m)
    {}

    bool                              incomingp;
    const hwm::lane*                  lane;
    const hwm::lane::road_membership* membership;
};

struct road_winding
{
    road_winding(const hwm::road *r, bool oa, float th)
        : road(r), oriented_away(oa), theta(th)
    {}

    const hwm::road *road;
    bool             oriented_away;
    float            theta;
};

struct circle_sort
{
    bool operator()(const road_winding &l, const road_winding &r) const
    {
        return l.theta < r.theta;
    }
};

void add_incident_lane_points(std::vector<vertex> &vrts, const oriented_membership &om, float sign)
{
    float param;
    float offs = sign*lane_width/2;
    if(om.incomingp)
        param = 1.0f;
    else
        param = 0.0f;

    {
        const vec3f pt(om.lane->point(param, offs));
        if(vrts.empty() || distance2(vrts.back().position, pt) > 1e-3)
            vrts.push_back(vertex(pt, vec3f(0.0, 0.0, 1.0), vec2f(0.0, 0.0)));
    }
    {
        const vec3f pt(om.lane->point(param, -offs));
        vrts.push_back(vertex(pt, vec3f(0.0, 0.0, 1.0), vec2f(0.0, 0.0)));
    }
}

void obj_intersection(std::ostream &os, const hwm::intersection &is)
{
    typedef std::map<const float, const oriented_membership> incident_members;
    typedef std::map<const hwm::road*, incident_members> road_member_map;

    road_member_map rmm;
    BOOST_FOREACH(const hwm::lane *incl, is.incoming)
    {
        const hwm::lane::road_membership *rm = &(boost::prior(incl->road_memberships.end())->second);
        const hwm::road *r = rm->parent_road;
        road_member_map::iterator ent(rmm.find(r));
        if(ent == rmm.end())
            ent = rmm.insert(ent, std::make_pair(r, incident_members()));
        ent->second.insert(std::make_pair(rm->lane_position, oriented_membership(true, incl, rm)));
    }
    BOOST_FOREACH(const hwm::lane *outl, is.outgoing)
    {
        const hwm::lane::road_membership *rm = &(outl->road_memberships.begin()->second);
        const hwm::road *r = rm->parent_road;
        road_member_map::iterator ent(rmm.find(r));
        if(ent == rmm.end())
            ent = rmm.insert(ent, std::make_pair(r, incident_members()));
        ent->second.insert(std::make_pair(rm->lane_position, oriented_membership(false, outl, rm)));
    }

    std::vector<road_winding> rw_sort;
    BOOST_FOREACH(const road_member_map::value_type &r, rmm)
    {
        bool oriented_away;
        vec2f avg(0.0f);
        BOOST_FOREACH(const incident_members::value_type &im, r.second)
        {
            const oriented_membership &om(im.second);
            oriented_away = (om.incomingp && (om.membership->interval[0] > om.membership->interval[1])) ||
                            (!om.incomingp && (om.membership->interval[0] < om.membership->interval[1]));
            const vec3f pt(om.lane->point(om.incomingp ? 1.0f : 0.0) - is.center);
            avg[0] += pt[0];
            avg[1] += pt[1];
        }

        rw_sort.push_back(road_winding(r.first, oriented_away, std::fmod(std::atan2(avg[1], avg[0])+M_PI, 2*M_PI)));
    }

    std::sort(rw_sort.begin(), rw_sort.end(), circle_sort());
    std::vector<vertex> vrts;
    for(size_t i = 0; i < rw_sort.size(); ++i)
    {
        vec3f start_point;
        vec3f start_tan;
        {
            const road_member_map::const_iterator  im0_itr = rmm.find(rw_sort[i].road);
            const incident_members                &im0     = im0_itr->second;

            const oriented_membership *first0;
            float                      orientation0;
            if(rw_sort[i].oriented_away)
            {
                orientation0 = -1.0;
                first0 = &(boost::prior(im0.end())->second);
                BOOST_FOREACH(const incident_members::value_type &im, im0)
                {
                    add_incident_lane_points(vrts, im.second, orientation0);
                }
            }
            else
            {
                orientation0 = 1.0;
                first0 = &(im0.begin()->second);
                BOOST_REVERSE_FOREACH(const incident_members::value_type &im, im0)
                {
                    add_incident_lane_points(vrts, im.second, orientation0);
                }
            }

            {
                float param;
                float tan_sign;
                if(first0->incomingp)
                {
                    param    = 1.0f;
                    tan_sign = 1.0f;
                }
                else
                {
                    param    =  0.0f;
                    tan_sign = -1.0f;
                }

                const mat4x4f start(first0->lane->point_frame(param, -tan_sign*orientation0*lane_width/2));
                for(int i = 0; i < 3; ++i)
                {
                    start_point[i] = start(i, 3);
                    start_tan[i]   = tan_sign*start(i, 0);
                }
            }
        }
        vec3f end_point;
        vec3f end_tan;
        {
            const road_member_map::const_iterator  im1_itr = rmm.find(rw_sort[(i+1)%rw_sort.size()].road);
            const incident_members                &im1     = im1_itr->second;

            const oriented_membership *last1;
            float                      orientation1;
            if(rw_sort[(i+1)%rw_sort.size()].oriented_away)
            {
                orientation1 = -1.0;
                last1 = &(im1.begin()->second);
            }
            else
            {
                orientation1 = 1.0;
                last1 = &(boost::prior(im1.end())->second);
            }

            {
                float param;
                float tan_sign;
                if(last1->incomingp)
                {
                    param    = 1.0f;
                    tan_sign = 1.0f;
                }
                else
                {
                    param    =  0.0f;
                    tan_sign = -1.0f;
                }

                const mat4x4f end(last1->lane->point_frame(param, -tan_sign*orientation1*lane_width/2));
                for(int i = 0; i < 3; ++i)
                {
                    end_point[i] = end(i, 3);
                    end_tan[i]   = tan_sign*end(i, 0);
                }
            }
        }

        arc_road ar;
        ar.initialize_from_polyline(0.0f, from_tan_pairs(start_point,
                                                         start_tan,
                                                         end_point,
                                                         end_tan,
                                                         0.0f));

        ar.extract_line(vrts, vec2f(0.0f, 1.0f), 0.0f, 0.01);
    }

    vrts.push_back(vertex(is.center, vec3f(0, 0, 1), vec2f(0.0, 0.0)));
    unsigned int center = static_cast<unsigned int>(vrts.size()-1);
    std::vector<vec3u> fcs;
    for(unsigned int i = 0; i < center-1; ++i)
        fcs.push_back(vec3u(center, i, (i+1) % center));

    dump_obj(os, is.id, "none", vrts, fcs);
}

int main(int argc, char *argv[])
{
    std::cerr << libroad_package_string() << std::endl;

    if(argc < 3)
    {
        std::cerr << "Usage: " << argv[0] << " <input network> <output obj>" << std::endl;
        return 1;
    }
    hwm::network net(hwm::load_xml_network(argv[1], vec3f(1.0, 1.0, 1.0f)));

    net.build_intersections();
    net.build_fictitious_lanes();
    net.auto_scale_memberships();
    net.center();
    std::cerr << "HWM net loaded successfully" << std::endl;

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

    bf::path full_out_path(argv[2]);

    bf::path dir(full_out_path.parent_path());
    bf::path tex_dir(full_out_path.parent_path() / "tex");

    if(!dir.empty())
       bf::create_directory(dir);
    bf::create_directory(tex_dir);

    if(!dir.empty())
        bf::current_path(dir);
    std::ofstream out(full_out_path.filename().c_str());
    obj_roads(out, net);

    BOOST_FOREACH(const hwm::intersection_pair &ip, net.intersections)
    {
        obj_intersection(out, ip.second);
    }

    return 0;
}
