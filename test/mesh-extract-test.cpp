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

int main(int argc, char *argv[])
{
    std::cerr << libroad_package_string() << std::endl;

    FIBITMAP *im = create_lane_image(true, 2, 1, true);
    FreeImage_Save(FIF_PNG, im, "test.png");
    FreeImage_Unload(im);

    arc_road ar;
    ar.points_.push_back(vec3f(00.0, 40.0, 0.0));
    ar.points_.push_back(vec3f(40.0, 30.0, 0.0));
    ar.points_.push_back(vec3f(40.0, 00.0, 0.0));
    ar.points_.push_back(vec3f(60.0, 00.0, 0.0));
    ar.points_.push_back(vec3f(30.0, -20.0, 0.0));
    ar.points_.push_back(vec3f(20.0, -10.0, 1.0));
    ar.points_.push_back(vec3f(20.0, -40.0, 1.0));
    ar.points_.push_back(vec3f(00.5, -20.0, 1.0));
    ar.points_.push_back(vec3f(00.5, -50.0, 2.0));
    ar.points_.push_back(vec3f(10.0, -70.0, 2.0));
    ar.points_.push_back(vec3f(00.0, -90, 2.0));
    ar.points_.push_back(vec3f(50.0, -80, 3.0));
    ar.points_.push_back(vec3f(70.5, -50, 3.0));
    ar.points_.push_back(vec3f(100, - 30, 3.0));
    ar.initialize_from_polyline(0.7f, ar.points_);

    std::vector<vertex> vrts;
    std::vector<vec3u>  fcs;

    ar.make_mesh(vrts, fcs, vec2f(0.0f, 1.0f), vec2f(-2.5f, 2.5f), 1.0f, true);

    dump_obj(std::cout, vrts, fcs);

    return 0;
}
