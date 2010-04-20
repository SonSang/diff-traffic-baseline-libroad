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

typedef tvmet::Vector<double, 4> color4d;

struct fill_box
{
    fill_box(const double   w_,  const double h_,
             const double   xr_, const double yr_,
             const color4d &c_)
        : w(w_), h(h_),
          xr(xr_), yr(yr_),
          c(c_)
    {}

    double width() const
    {
        return w;
    }

    double height() const
    {
        return h;
    }

    double xres() const
    {
        return xr;
    }

    double yres() const
    {
        return yr;
    }

    void draw(cairo_t *cr) const
    {
        cairo_set_source_rgba(cr, c[0], c[1], c[2], c[3]);
        cairo_rectangle(cr, 0, 0.0, w, yr);
        cairo_fill(cr);
    }

    double  w, h;
    double  xr, yr;
    color4d c;
};

struct lane_maker
{
    void res_scale()
    {
        double total_w    = 0.0;
        double min_x_feat = std::numeric_limits<double>::max();
        double max_h      = 0.0;
        double y_res      = 0.0;
        BOOST_FOREACH(const fill_box &fb, boxes)
        {
            total_w    += fb.width();
            min_x_feat  = std::min(min_x_feat, fb.xres());
            max_h       = std::max(max_h, fb.height());
            y_res       = std::max(y_res, fb.height()/fb.yres());
        }

        im_res = 100*vec2u(static_cast<unsigned int>(std::ceil(total_w/min_x_feat)),
                           static_cast<unsigned int>(std::ceil(y_res)));
        scale  = vec2d(total_w, max_h);
    }

    void draw(const std::string &fname) const
    {
        cairo_surface_t *cs = cairo_image_surface_create(CAIRO_FORMAT_ARGB32,
                                                         im_res[0],
                                                         im_res[1]);
        cairo_t         *cr = cairo_create(cs);

        cairo_set_source_rgba(cr, 0, 0, 0, 0);
        cairo_set_operator(cr, CAIRO_OPERATOR_SOURCE);
        cairo_paint(cr);

        cairo_set_operator(cr, CAIRO_OPERATOR_OVER);
        std::cout << scale << std::endl;
        std::cout << im_res << std::endl;
        cairo_scale(cr, im_res[0]/scale[0], im_res[1]/scale[1]);

        BOOST_FOREACH(const fill_box &fb, boxes)
        {
            fb.draw(cr);
            cairo_translate(cr, fb.width(), 0.0);
        }

        cairo_destroy(cr);
        cairo_surface_write_to_png(cs, fname.c_str());
        cairo_surface_destroy(cs);
    }

    vec2u                 im_res;
    vec2d                 scale;
    std::vector<fill_box> boxes;
};
#endif
