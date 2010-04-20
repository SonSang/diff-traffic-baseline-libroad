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

struct lane_op
{
    virtual double width() const         = 0;
    virtual double xres() const          = 0;
    virtual double height() const        = 0;
    virtual double yres() const          = 0;
    virtual void draw(cairo_t *cr) const = 0;
};

struct xgap : public lane_op
{
    xgap(double  w_)
        : w(w_)
    {}

    virtual double width() const
    {
        return w;
    }

    virtual double xres() const
    {
        return w;
    }

    virtual double height() const
    {
        return 0.0;
    }

    virtual double yres() const
    {
        return std::numeric_limits<double>::max();
    }

    virtual void draw(cairo_t *cr) const
    {
    }

    double w;
};

struct fill_box : public lane_op
{
    fill_box(double  w_,     double h_,
             double  ybase_, double ylen_,
             color4d c_) : w(w_), h(h_),
                           ybase(ybase_), ylen(ylen_),
                           c(c_)
    {}

    virtual double width() const
    {
        return w;
    }

    virtual double xres() const
    {
        return w;
    }

    virtual double height() const
    {
        return h;
    }

    virtual double yres() const
    {
        const double base = ybase > 0.0 ? ybase : std::numeric_limits<double>::max();
        const double top  = (h-ybase-ylen) > 0.0 ? h-ybase-ylen : std::numeric_limits<double>::max();
        return std::min(base, std::min(ylen, top));
    }

    virtual void draw(cairo_t *cr) const
    {
        cairo_set_source_rgba(cr, c[0], c[1], c[2], c[3]);
        cairo_rectangle(cr, 0, ybase, w, ylen);
        cairo_fill(cr);
    }

    double  w, h;
    double  ybase;
    double  ylen;
    color4d c;
};

struct lane_maker
{
    void res_scale()
    {
        double total_w    = 0.0;
        double min_x_feat = std::numeric_limits<double>::max();
        double max_h      = 0.0;
        double min_y_feat = std::numeric_limits<double>::max();
        BOOST_FOREACH(const lane_op *lo, boxes)
        {
            total_w    += lo->width();
            min_x_feat  = std::min(min_x_feat, lo->xres());
            max_h       = std::max(max_h, lo->height());
            min_y_feat  = std::min(min_y_feat, lo->yres());
        }
        if(max_h == 0.0)
            max_h = 1.0;

        im_res = 100*vec2u(static_cast<unsigned int>(std::ceil(total_w/min_x_feat)),
                           static_cast<unsigned int>(std::ceil(max_h/min_y_feat)));
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

        BOOST_FOREACH(const lane_op *lo, boxes)
        {
            lo->draw(cr);
            cairo_translate(cr, lo->width(), 0.0);
        }

        cairo_destroy(cr);
        cairo_surface_write_to_png(cs, fname.c_str());
        cairo_surface_destroy(cs);
    }

    vec2u                 im_res;
    vec2d                 scale;
    std::vector<lane_op*> boxes;
};
#endif
