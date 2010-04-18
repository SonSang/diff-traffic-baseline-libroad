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

    // for(unsigned int y = 0; y < FreeImage_GetHeight(im); y++)
    // {
    //     BYTE *bits = FreeImage_GetScanLine(im, y);
    //     for(unsigned int x = 0; x < FreeImage_GetWidth(im); x++)
    //     {
    //         bits[FI_RGBA_ALPHA] = 255;
    //         bits += bytespp;
    //     }
    // }

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

    return im;
}

struct lane_tex
{
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

    std::string tex_string(bool lshoulder, int llanes, int rlanes, bool rshoulder)
    {
        canon(lshoulder, llanes, rlanes, rshoulder);
        return boost::str(boost::format("ls%d-ll%d-rl%d-rs%d") % lshoulder % llanes % rlanes % rshoulder);
    }

    std::string tex_path(const std::string &s)
    {
        return boost::str(boost::format("%s%s.png") % root % s);
    }

    const std::string write_tex(const bool lshoulder, const int llanes, const int rlanes, const bool rshoulder)
    {
        const int idx(hash_index(lshoulder, llanes, rlanes, rshoulder));
        std::map<const int, const std::string>::iterator c(texes.find(idx));
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

    std::map<const int, const std::string> texes;
    std::string    root;
};

int main(int argc, char *argv[])
{
    std::cerr << libroad_package_string() << std::endl;

    lane_tex ltb("tex/");

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


    std::cout << "mtllib road.mtl\n";
    BOOST_FOREACH(const hwm::road_pair &r, net.roads)
    {
        r.second.rep.make_mesh(vrts, fcs, vec2f(0.0f, 1.0f), vec2f(-2*2.5f, 2*2.5f), 1.0);

        dump_obj(std::cout,
                 r.first,
                 ltb.write_tex(true, 2, 2, true),
                 vrts,
                 last_v, vrts.size(),
                 fcs,
                 last_f, fcs.size());

        last_v = vrts.size();
        last_f = fcs.size();
    }

    return 0;
}
