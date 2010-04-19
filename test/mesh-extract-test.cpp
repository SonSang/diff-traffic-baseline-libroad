#include "libroad/hwm_network.hpp"

#include <fstream>
#include <FreeImage.h>

void dump_obj(std::ostream      &out,
              const std::string &name,
              const std::string &material_name,
              const std::vector<vertex> &verts,
              size_t vert_start, size_t vert_end,
              const std::vector<vec3u>  &faces,
              size_t face_start, size_t face_end)
{
    out << "o " << name << "\n";
    out << "usemtl " << material_name << "\n";
    for(size_t i = vert_start; i < vert_end; ++i)
    {
        const vertex &v = verts[i];
        out << "v " << v.position[0] << " " << v.position[1] << " " << v.position[2]  << "\n"
            << "vn " << v.normal[0] << " " << v.normal[1] << " " << v.normal[2]  << "\n"
            << "vt " << v.tex_coord[0] << " " << v.tex_coord[1] << "\n";
    }

    for(size_t i = face_start; i < face_end; ++i)
    {
        const vec3u &f = faces[i];
        out << "f ";
        BOOST_FOREACH(const unsigned int i, f)
        {
            out << i+1 << "/" << i+1 << "/" << i+1 << " ";
        }
        out << "\n";
    }
};

static const float lane_width      = 2.5f;
static const float shoulder_width  = 2.0f;
static const float line_width      = 0.125;
static const float line_sep_width  = 0.125;
static const float line_length     = 3.0f;
static const float line_gap_length = 9.0f;

void set_dashed(FIBITMAP *im, const int y, const int bytespp)
{
    BYTE *bits = FreeImage_GetScanLine(im, y);
    for(unsigned int x = static_cast<unsigned int>(std::floor(line_gap_length/(line_gap_length+line_length)*FreeImage_GetWidth(im))); x < FreeImage_GetWidth(im); x++)
    {
        bits[FI_RGBA_RED]   = 255;
        bits[FI_RGBA_GREEN] = 255;
        bits[FI_RGBA_BLUE] =  255;
        bits[FI_RGBA_ALPHA] = 255;
        // jump to next pixel
        bits += bytespp;
    }
}

void set_solid_white(FIBITMAP *im, const int y, const int bytespp)
{
    BYTE *bits = FreeImage_GetScanLine(im, y);
    for(unsigned int x = 0; x <  FreeImage_GetWidth(im); x++)
    {
        bits[FI_RGBA_RED]   = 255;
        bits[FI_RGBA_GREEN] = 255;
        bits[FI_RGBA_BLUE] =  255;
        bits[FI_RGBA_ALPHA] = 255;
        // jump to next pixel
        bits += bytespp;
    }
}

void set_solid_yellow(FIBITMAP *im, const int y, const int bytespp)
{
    BYTE *bits = FreeImage_GetScanLine(im, y);
    for(unsigned int x = 0; x < FreeImage_GetWidth(im); x++)
    {
        bits[FI_RGBA_RED]   = 0xFF;
        bits[FI_RGBA_GREEN] = 0xe7;
        bits[FI_RGBA_BLUE] =  0x00;
        bits[FI_RGBA_ALPHA] = 255;
        // jump to next pixel
        bits += bytespp;
    }
}

FIBITMAP *create_lane_image(bool lshoulder, int llanes, int rlanes, bool rshoulder)
{
    const float total_length = (lshoulder + rshoulder)*shoulder_width
                               + lane_width * (llanes + rlanes);

    const int xres = static_cast<int>(std::ceil((line_gap_length+line_length)/line_length));
    const int yres = static_cast<int>(std::ceil(total_length/line_sep_width));

    FIBITMAP *im = FreeImage_Allocate(xres, yres, 32, FI_RGBA_RED_MASK, FI_RGBA_GREEN_MASK, FI_RGBA_BLUE_MASK);
    assert(im);
    const int bytespp = FreeImage_GetLine(im) / FreeImage_GetWidth(im);

    for(unsigned int y = 0; y < FreeImage_GetHeight(im); y++)
    {
        BYTE *bits = FreeImage_GetScanLine(im, y);
        for(unsigned int x = 0; x < FreeImage_GetWidth(im); x++)
        {
            bits[FI_RGBA_ALPHA] = 255;
            bits += bytespp;
        }
    }

    float current = 0.0f;
    if(lshoulder)
    {
        current += (shoulder_width-line_width*0.5);
        set_solid_white(im, current/total_length*FreeImage_GetHeight(im), bytespp);
        current += line_width*0.5f;
    }

    for(int i = llanes; i > 1; --i)
    {
        current += (lane_width-line_width*0.5);
        set_dashed(im, current/total_length*FreeImage_GetHeight(im), bytespp);
        current += line_width*0.5f;
    }

    if(llanes > 0)
    {
        current += (lane_width-line_width);
        set_solid_yellow(im, current/total_length*FreeImage_GetHeight(im), bytespp);
        current += line_width;
    }

    if(rlanes > 0)
    {
        current += line_width;
        set_solid_yellow(im, current/total_length*FreeImage_GetHeight(im), bytespp);
        current += lane_width - line_width;
    }

    for(int i = 1; i < rlanes; ++i)
    {
        current -= line_width*0.5;
        set_dashed(im, current/total_length*FreeImage_GetHeight(im), bytespp);
        current += lane_width - line_width*0.5f;
    }

    if(rshoulder)
    {
        current -= line_width*0.5;
        set_solid_white(im, current/total_length*FreeImage_GetHeight(im), bytespp);
    }

    im = FreeImage_Rescale(im, 5*FreeImage_GetWidth(im), 5*FreeImage_GetHeight(im), FILTER_BOX);

    return im;
}

struct lane_tex
{
    typedef std::map<const int, const std::string> str_map;


    lane_tex(const std::string &r) : root(r) {}

    static const int max_lanes = 1024;
    static void canon(bool &lshoulder, int &llanes, int &rlanes, bool &rshoulder)
    {
        if(lshoulder > rshoulder)
            std::swap(lshoulder, rshoulder);
        if(llanes > rlanes)
            std::swap(llanes, rlanes);
    }
    static int hash_index(bool lshoulder, int llanes, int rlanes, bool rshoulder)
    {
        canon(lshoulder, llanes, rlanes, rshoulder);
        return (llanes * 1024 + rlanes) * 1024 +  lshoulder *2 + rshoulder;
    }

    static std::string tex_string(bool lshoulder, int llanes, int rlanes, bool rshoulder)
    {
        canon(lshoulder, llanes, rlanes, rshoulder);
        return boost::str(boost::format("ls%d-ll%d-rl%d-rs%d") % lshoulder % llanes % rlanes % rshoulder);
    }

    std::string tex_path(const std::string &s) const
    {
        return boost::str(boost::format("%s%s.png") % root % s);
    }

    const std::string write_tex(const bool lshoulder, const int llanes, const int rlanes, const bool rshoulder)
    {
        const int idx(hash_index(lshoulder, llanes, rlanes, rshoulder));
        const str_map::iterator c(texes.find(idx));
        if(c == texes.end())
        {
            FIBITMAP *im = create_lane_image(lshoulder, llanes, rlanes, rshoulder);
            const std::string fi(tex_string(lshoulder, llanes, rlanes, rshoulder));
            FreeImage_Save(FIF_PNG, im, tex_path(fi).c_str());
            texes.insert(c, std::make_pair(idx, fi));
            FreeImage_Unload(im);
            return fi;
        }
        return c->second;
    }

    void write_mtl(std::ostream &o,
                   const std::string &ts) const
    {
        o << boost::str(boost::format("newmtl %s\n"
                                      "ns 96.078431\n"
                                      "ka 0.0  0.0  0.0\n"
                                      "kd 0.64 0.64 0.64\n"
                                      "ks 0.5  0.5  0.5\n"
                                      "ni 1.0\n"
                                      "d  1.0\n"
                                      "map_kd %s\n") % ts % tex_path(ts));
    }

    void write_mtllib(const std::string &fname) const
    {
        std::ofstream o(fname.c_str());

        BOOST_FOREACH(const str_map::value_type &i, texes)
        {
            write_mtl(o, i.second);
        }
    }

    std::map<const int, const std::string> texes;
    std::string    root;
};

struct road_rev_map
{
    struct lane_entry
    {
        lane_entry()
        {}

        lane_entry(const hwm::lane* l, bool dw) :
            lane(l), dir_with(dw)
        {}

        const hwm::lane *lane;
        bool dir_with;
    };

    typedef std::vector<lane_entry>  lane_cont;
    partition01<lane_cont>           lane_map;
    const hwm::road                 *road;

    road_rev_map()
    {
        lane_map.insert(0.0f, lane_cont());
    }

    road_rev_map(const hwm::road *r) : road(r)
    {
        lane_map.insert(0.0f, lane_cont());
    }

    void add_lane(const hwm::lane *r, const vec2f &iv)
    {
        vec2f interval(iv);
        lane_entry le(r, true);
        if(iv[0] > iv[1])
        {
            std::swap(interval[0], interval[1]);
            le.dir_with = false;
        }

        partition01<lane_cont>::iterator start(lane_map.find(interval[0]));
        const vec2f         start_interval(lane_map.containing_interval(start));

        start = lane_map.split_interval(start, vec2f(interval[0], std::min(interval[1], start_interval[1])), start->second);

        start->second.push_back(le);
        if(interval[1] <= start_interval[1])
            return;

        partition01<lane_cont>::iterator end(lane_map.find(interval[1]));
        const vec2f         end_interval(lane_map.containing_interval(end));

        end = lane_map.split_interval(end, vec2f(end_interval[0], interval[1]), end->second);
        end->second.push_back(le);

        for(partition01<lane_cont>::iterator current = boost::next(start); current != end; ++current)
            current->second.push_back(le);
    }

    void print() const
    {
        for(partition01<lane_cont>::const_iterator current = lane_map.begin(); current != lane_map.end(); ++current)
        {
            std::cout << lane_map.containing_interval(current) << ": ";
            BOOST_FOREACH(const lane_entry &le, current->second)
            {
                std::cout << le.lane->id;
                if(le.dir_with)
                    std::cout << "(+) ";
                else
                    std::cout << "(-) ";

            }
            std::cout << std::endl;
        }
    }
};

int main(int argc, char *argv[])
{
    std::cerr << libroad_package_string() << std::endl;

    if(argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " <input network>" << std::endl;
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

    std::vector<vertex> vrts;
    std::vector<vec3u>  fcs;
    size_t              last_v = 0;
    size_t              last_f = 0;

    strhash<road_rev_map>::type rrm;

    BOOST_FOREACH(const hwm::road_pair &r, net.roads)
    {
        rrm[r.first] = road_rev_map(&(r.second));
    }

    BOOST_FOREACH(const hwm::lane_pair &l, net.lanes)
    {
        BOOST_FOREACH(const hwm::lane::road_membership::intervals::entry &rm, l.second.road_memberships)
        {
            rrm[rm.second.parent_road->id].add_lane(&(l.second), rm.second.interval);
        }
    }

    lane_tex ltb("tex/");
    std::cout << "mtllib road.mtl\n";
    BOOST_FOREACH(const strhash<road_rev_map>::type::value_type &rrm_v, rrm)
    {
        const hwm::road &r = *(rrm_v.second.road);
        size_t re_c = 0;
        for(partition01<road_rev_map::lane_cont>::const_iterator current = rrm_v.second.lane_map.begin();
            current != rrm_v.second.lane_map.end();
            ++current)
        {
            const road_rev_map::lane_cont &e = current->second;
            size_t against = 0;
            size_t with    = 0;
            BOOST_FOREACH(const road_rev_map::lane_entry &le, e)
            {
                if(le.dir_with)
                    ++with;
                else
                    ++against;
            }

            r.rep.make_mesh(vrts, fcs, rrm_v.second.lane_map.containing_interval(current), vec2f(-against*2.5f, with*2.5f), 0.01);

            dump_obj(std::cout,
                     boost::str(boost::format("%s-%d") % r.id % re_c),
                     ltb.write_tex(false, against, with, false),
                     vrts,
                     last_v, vrts.size(),
                     fcs,
                     last_f, fcs.size());

            last_v = vrts.size();
            last_f = fcs.size();
            ++re_c;
        }
    }

    ltb.write_mtllib("road.mtl");

    return 0;
}
