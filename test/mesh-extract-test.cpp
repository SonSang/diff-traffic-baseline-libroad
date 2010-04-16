#include "libroad/arc_road.hpp"

#include <fstream>
#include <FreeImage.h>

void dump_obj(std::ostream &out, const std::vector<vertex> &verts,
              std::vector<vec3u> &faces)
{
    BOOST_FOREACH(const vertex &v, verts)
    {
        out << "v " << v.position[0] << " " << v.position[1] << " " << v.position[2]  << "\n"
            << "vn " << v.normal[0] << " " << v.normal[1] << " " << v.normal[2]  << "\n"
            << "vt " << v.tex_coord[0] << " " << v.tex_coord[1] << "\n";
    }

    BOOST_FOREACH(const vec3u &f, faces)
    {
        out << "f ";
        BOOST_FOREACH(const unsigned int i, f)
        {
            out << i+1 << "/" << i+1 << "/" << i+1 << " ";
        }
        out << "\n";
    }
};

static const float lane_width      = 2.5f;
static const float shoulder_width  = 3.5f;
static const float line_width      = 0.125;
static const float line_sep_width  = 0.125;
static const float line_length     = 3.0f;
static const float line_gap_length = 9.0f;

void create_lane_image(bool lshoulder, int llanes, int rlanes, bool rshoulder)
{
    const float total_length = (lshoulder + rshoulder)*shoulder_width
                                      + lane_width * (llanes + rlanes);

    const int xres = static_cast<int>(std::ceil(total_length/line_sep_width));
    const int yres = static_cast<int>(std::ceil((line_gap_length+line_length)/line_length));

    FIBITMAP *im = FreeImage_Allocate(100, 100, 32, FI_RGBA_RED_MASK, FI_RGBA_GREEN_MASK, FI_RGBA_BLUE_MASK);
    assert(im);
    const int bytespp = FreeImage_GetLine(im) / FreeImage_GetWidth(im);

    for(unsigned int y = 0; y < FreeImage_GetHeight(im); y++)
    {
        BYTE *bits = FreeImage_GetScanLine(im, y);
        for(unsigned int x = 0; x <  FreeImage_GetWidth(im); x++)
        {
            bits[FI_RGBA_RED]   = 255;
            bits[FI_RGBA_GREEN] = 0;
            bits[FI_RGBA_BLUE] =  0;
            bits[FI_RGBA_ALPHA] = 255;
            // jump to next pixel
            bits += bytespp;
        }
    }

    FreeImage_Save(FIF_PNG, im, "test.png");

    FreeImage_Unload(im);
}

int main(int argc, char *argv[])
{
    std::cerr << libroad_package_string() << std::endl;

    create_lane_image(false, 1, 1, false);

    arc_road ar;
    ar.points_.push_back(vec3f(000, 400, 0.0));
    ar.points_.push_back(vec3f(400, 300, 0.0));
    ar.points_.push_back(vec3f(400, 000, 0.0));
    ar.points_.push_back(vec3f(600, 000, 0.0));
    ar.points_.push_back(vec3f(300, -200, 0.0));
    ar.points_.push_back(vec3f(200, -100, 10));
    ar.points_.push_back(vec3f(200, -400, 10));
    ar.points_.push_back(vec3f(005, -200, 10));
    ar.points_.push_back(vec3f(005, -500, 20));
    ar.points_.push_back(vec3f(100, -700, 20));
    ar.points_.push_back(vec3f(000, -900, 20));
    ar.points_.push_back(vec3f(500, -800, 30));
    ar.points_.push_back(vec3f(705, -500, 30));
    ar.points_.push_back(vec3f(1000, -300, 30));
    ar.initialize_from_polyline(0.7f, ar.points_);

    std::vector<vertex> vrts;
    std::vector<vec3u>  fcs;

    ar.make_mesh(vrts, fcs, vec2f(0.0f, 1.0f), vec2f(-2*2.5f, 2*2.5f), 10.0f);

    dump_obj(std::cout, vrts, fcs);

    return 0;
}
