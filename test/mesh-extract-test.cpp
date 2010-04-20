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
            lm.add_xgap(0.25*lane_width);
            lm.add_cbox(new single_box(line_width, line_length+line_gap_length,
                                       0,          line_length+line_gap_length,
                                       color4d(1.0, 1.0, 1.0, 1.0)));

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
            lm.add_cbox(new single_box(line_width, line_length+line_gap_length,
                                       0,          line_length+line_gap_length,
                                       color4d(1.0, 1.0, 1.0, 1.0)));
            lm.add_xgap(0.25*lane_width);

            lm.draw(texfile);
        }

        void make_mesh(std::vector<vertex> &vrts, std::vector<vec3u> &fcs, const vec2f &interval) const
        {
            const float left  = begin()            ->first-0.5f*lane_width-0.25f*lane_width;
            const float right = boost::prior(end())->first+0.5f*lane_width+0.25f*lane_width;
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

    return 0;
}
