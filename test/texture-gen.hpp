#ifndef _TEXTURE_GEN_HPP_
#define _TEXTURE_GEN_HPP_

#include <fstream>
#include <cairo.h>
#include <FreeImage.h>

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

#endif
