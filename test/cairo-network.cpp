#include "libroad/hwm_network.hpp"

int main(int argc, char *argv[])
{
    std::cout << libroad_package_string() << std::endl;
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

    vec2i im_res(2000, 2000);
    cairo_surface_t *cs = cairo_image_surface_create(CAIRO_FORMAT_ARGB32,
                                                     im_res[0],
                                                     im_res[1]);
    cairo_t         *cr = cairo_create(cs);

    cairo_set_source_rgba(cr, 0, 0, 0, 1.0);
    cairo_set_operator(cr, CAIRO_OPERATOR_SOURCE);
    cairo_paint(cr);

    cairo_set_operator(cr, CAIRO_OPERATOR_OVER);
    vec3f low(FLT_MAX);
    vec3f high(-FLT_MAX);
    net.bounding_box(low, high);
    vec3f dv(high-low);
    low  -= dv*0.1;
    high += dv*0.1;
    dv   *= 1.2;

    const float view_aspect  = static_cast<float>(im_res[0])/im_res[1];
    const float scene_aspect = dv[0]/dv[1];

    float scale;
    if(view_aspect > scene_aspect)
        scale = im_res[1]/dv[1];
    else
        scale = im_res[0]/dv[0];

    cairo_scale(cr,
                scale,
                -scale);
    cairo_translate(cr,
                    -low[0],
                    low[1]);

    cairo_set_line_width(cr, 0.5);
    cairo_set_source_rgba(cr, 1.0, 1.0, 1.0, 1.0);

    hwm::network_aux neta(net);
    neta.cairo_roads(cr);

    cairo_destroy(cr);
    cairo_surface_write_to_png(cs, "test2d.png");
    cairo_surface_destroy(cs);


    return 0;
}
