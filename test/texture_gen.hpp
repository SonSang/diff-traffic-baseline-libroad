#ifndef _TEXTURE_GEN_HPP_
#define _TEXTURE_GEN_HPP_

#include <fstream>
#include <cairo.h>

static const float lane_width      = 2.5f;
static const float shoulder_width  = 2.0f;
static const float line_width      = 0.125;
static const float line_sep_width  = 0.125;
static const float line_length     = 3.0f;
static const float line_gap_length = 9.0f;

typedef tvmet::Vector<double, 4> color4d;

struct lane_op
{
    virtual std::string rep() const      = 0;
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

    virtual std::string rep() const
    {
        return boost::str(boost::format("xg%f") % w);
    }

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

void aligned_rectangle(cairo_t *cr, double x, double y, double w, double h)
{
    cairo_user_to_device(cr, &x, &y);
    x = std::floor(x);
    y = std::floor(y);
    cairo_device_to_user(cr, &x, &y);

    w += x;
    h += y;
    cairo_user_to_device(cr, &w, &h);
    w = std::ceil(w);
    h = std::ceil(h);
    cairo_device_to_user(cr, &w, &h);

    cairo_rectangle(cr, x, y, w-x, h-y);
}

struct center_box : public lane_op
{
    virtual std::string rep() const      = 0;
    virtual double width() const         = 0;
    virtual double xres() const          = 0;
    virtual double height() const        = 0;
    virtual double yres() const          = 0;
    virtual void draw(cairo_t *cr) const = 0;
};

struct single_box : public center_box
{
    single_box(double  w_,     double h_,
               double  ybase_, double ylen_,
               color4d c_) : w(w_), h(h_),
                             ybase(ybase_), ylen(ylen_),
                             c(c_)
    {}

    virtual std::string rep() const
    {
        return boost::str(boost::format("sb%f-%f-%f-%f-%f-%f-%f-%f") % w % h % ybase % ylen % c[0] % c[1] % c[2] % c[3]);
    }

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
        aligned_rectangle(cr, ybase, 0, ylen, w);
        cairo_fill(cr);
    }

    double  w, h;
    double  ybase;
    double  ylen;
    color4d c;
};

struct double_box : public center_box
{
    double_box(double bw_, double sep_,
               double h_,  double ybase_, double ylen_,
               color4d c_) : bw(bw_), sep(sep_), h(h_),
                             ybase(ybase_), ylen(ylen_),
                             c(c_)
    {}

    virtual std::string rep() const
    {
        return boost::str(boost::format("db%f-%f-%f-%f-%f-%f-%f-%f-%f") % bw % sep % h % ybase % ylen % c[0] % c[1] % c[2] % c[3]);
    }

    virtual double width() const
    {
        return bw*2 + sep;
    }

    virtual double xres() const
    {
        return std::min(sep, bw);
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

        aligned_rectangle(cr, ybase, 0, ylen, bw);
        cairo_fill(cr);

        aligned_rectangle(cr, ybase, sep+bw, ylen, bw);
        cairo_fill(cr);
    }

    double  bw, sep, h;
    double  ybase;
    double  ylen;
    color4d c;
};

struct lane_maker
{
    ~lane_maker()
    {
        BOOST_FOREACH(lane_op *o, boxes)
        {
            delete o;
        }
    }

    void add_cbox(center_box *cb)
    {
        if(!boxes.empty())
        {
            xgap *b = dynamic_cast<xgap*>(boxes.back());
            if(b)
                b->w -= cb->width()*0.5;
        }
        boxes.push_back(cb);
    }

    void add_xgap(double w)
    {
        if(!boxes.empty())
        {
            center_box *b = dynamic_cast<center_box*>(boxes.back());
            if(b)
                w -= b->width()*0.5;
        }
        boxes.push_back(new xgap(w));
    }

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

        im_res = 100*vec2u(static_cast<unsigned int>(std::ceil(max_h/min_y_feat)),
                           static_cast<unsigned int>(std::ceil(total_w/min_x_feat)));
        scale  = vec2d(max_h, total_w);
    }

    void draw(const std::string &fname)
    {
        res_scale();
        cairo_surface_t *cs = cairo_image_surface_create(CAIRO_FORMAT_ARGB32,
                                                         im_res[0],
                                                         im_res[1]);
        cairo_t         *cr = cairo_create(cs);

        cairo_set_source_rgba(cr, 0, 0, 0, 1.0);
        cairo_set_operator(cr, CAIRO_OPERATOR_SOURCE);
        cairo_paint(cr);

        cairo_set_operator(cr, CAIRO_OPERATOR_OVER);
        cairo_scale(cr, im_res[0]/scale[0], im_res[1]/scale[1]);

        BOOST_FOREACH(const lane_op *lo, boxes)
        {
            lo->draw(cr);
            cairo_translate(cr, 0.0, lo->width());
        }

        cairo_destroy(cr);
        cairo_surface_write_to_png(cs, fname.c_str());
        cairo_surface_destroy(cs);
    }

    std::string make_string() const
    {
        std::string res;
        BOOST_FOREACH(const lane_op *lo, boxes)
        {
            res += lo->rep() + "!";
        }
        return res;
    }

    vec2u                 im_res;
    vec2d                 scale;
    std::vector<lane_op*> boxes;
};
#endif
