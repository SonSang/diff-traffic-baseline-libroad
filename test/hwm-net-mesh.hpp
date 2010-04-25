#ifndef __HWM_NET_MESH_HPP__
#define __HWM_NET_MESH_HPP__

#include "libroad/hwm_network.hpp"
#include "texture-gen.hpp"
#include "boost/filesystem.hpp"

namespace bf = boost::filesystem;

void write_road_mtl(std::ostream      &o,
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

            const std::string oname(boost::str(boost::format("%s-%d") % r.id % re_c));
            const std::string texfilename(bf::path(bf::current_path() / "tex" / boost::str(boost::format("%s.png") % oname)).string());

            e.write_texture(texfilename);
            mesh_to_obj(os,
                        oname,
                        oname,
                        vrts,
                        fcs);

            write_road_mtl(mtllib, oname, texfilename);
            ++re_c;
        }
    }
}

struct road_winding
{
    road_winding(const size_t rn, float th)
        : road_num(rn), theta(th)
    {}

    size_t road_num;
    float  theta;
};

struct circle_sort
{
    bool operator()(const road_winding &l, const road_winding &r) const
    {
        return l.theta < r.theta;
    }
};

struct point_tan
{
    vec3f point;
    vec3f tan;
};

typedef std::pair<float, point_tan> offs_pt;

void gen_tan_points(offs_pt &low, offs_pt &high,
                    const hwm::lane &l, const bool incomingp)
{
    float param;
    float tan_sign;
    if(incomingp)
    {
        param    = 1.0f;
        tan_sign = 1.0f;
    }
    else
    {
        param    =  0.0f;
        tan_sign = -1.0f;
    }

    low.first = -tan_sign*lane_width/2;
    const mat4x4f low_mat(l.point_frame(param, low.first));
    for(int i = 0; i < 3; ++i)
    {
        low.second.point[i] = low_mat(i, 3);
        low.second.tan[i]   = tan_sign*low_mat(i, 0);
    }

    high.first = tan_sign*lane_width/2;
    const mat4x4f high_mat(l.point_frame(param, high.first));
    for(int i = 0; i < 3; ++i)
    {
        high.second.point[i] = high_mat(i, 3);
        high.second.tan[i]   = tan_sign*high_mat(i, 0);
    }
}

typedef std::map<float, point_tan> incident_point_tans;

struct road_store
{
    road_store(const hwm::road *r, const bool as)
        : road(r), at_start(as)
    {}

    const hwm::road     *road;
    bool                 at_start;
    incident_point_tans  ipt;
};

struct road_is_cnt : public std::vector<road_store>
{
    road_store &find(const hwm::road *r, bool as)
    {
        BOOST_FOREACH(road_store &rw, *this)
        {
            if(rw.road == r && rw.at_start == as)
                return rw;
        }

        push_back(road_store(r, as));
        return back();
    };
};

void obj_intersection(std::ostream &os, const hwm::intersection &is)
{
    road_is_cnt ric;
    BOOST_FOREACH(const hwm::lane *incl, is.incoming)
    {
        const hwm::lane::road_membership *rm       = &(boost::prior(incl->road_memberships.end())->second);
        const hwm::road                  *r        = rm->parent_road;
        const bool                        at_start = (rm->interval[0] > rm->interval[1]);
        road_store                       &ent(ric.find(r, at_start));

        offs_pt low; offs_pt high;
        gen_tan_points(low, high, *incl, true);

        const float oriented_sign = at_start ? 1.0 : -1.0;

        low.first  = oriented_sign*(low.first + rm->lane_position);
        high.first = oriented_sign*(high.first + rm->lane_position);
        ent.ipt.insert(low);
        ent.ipt.insert(high);
    }

    BOOST_FOREACH(const hwm::lane *outl, is.outgoing)
    {
        const hwm::lane::road_membership *rm       = &(outl->road_memberships.begin()->second);
        const hwm::road                  *r        = rm->parent_road;
        const bool                        at_start = (rm->interval[0] < rm->interval[1]);
        road_store                       &ent(ric.find(r, at_start));

        offs_pt low; offs_pt high;
        gen_tan_points(low, high, *outl, false);

        const float oriented_sign = at_start ? 1.0 : -1.0;

        low.first  = oriented_sign*(low.first + rm->lane_position);
        high.first = oriented_sign*(high.first + rm->lane_position);
        ent.ipt.insert(low);
        ent.ipt.insert(high);
    }

    std::vector<road_winding> rw_sort;
    for(size_t road_num = 0; road_num < ric.size(); ++road_num)
    {
        const road_store &r = ric[road_num];

        vec2f avg(0.0f);
        BOOST_FOREACH(const offs_pt &op, r.ipt)
        {
            const vec3f svec(op.second.point - is.center);
            avg[0] += svec[0];
            avg[1] += svec[1];
        }
        rw_sort.push_back(road_winding(road_num, std::fmod(std::atan2(avg[1], avg[0])+M_PI, 2*M_PI)));
    }

    std::sort(rw_sort.begin(), rw_sort.end(), circle_sort());

    std::vector<vertex> vrts;
    for(size_t i = 0; i < rw_sort.size(); ++i)
    {
        BOOST_FOREACH(const offs_pt &op, ric[rw_sort[i].road_num].ipt)
        {
            vrts.push_back(vertex(op.second.point, vec3f(0, 0, 1), vec2f(0.0, 0.0)));
        }

        const point_tan &start= boost::prior(ric[rw_sort[i].road_num].ipt.end())->second;
        const point_tan &end  = ric[rw_sort[(i+1)%rw_sort.size()].road_num].ipt.begin()->second;

        arc_road ar;
        ar.initialize_from_polyline(0.0f, from_tan_pairs(start.point,
                                                         start.tan,
                                                         end.point,
                                                         end.tan,
                                                         0.0f));

        ar.extract_line(vrts, vec2f(0.0f, 1.0f), 0.0f, 0.01);
   }

    vrts.push_back(vertex(is.center, vec3f(0, 0, 1), vec2f(0.0, 0.0)));
    unsigned int center = static_cast<unsigned int>(vrts.size()-1);
    std::vector<vec3u> fcs;
    for(unsigned int i = 0; i < center; ++i)
        fcs.push_back(vec3u(center, i, (i+1) % center));

    mesh_to_obj(os, is.id, "none", vrts, fcs);
}
#endif
