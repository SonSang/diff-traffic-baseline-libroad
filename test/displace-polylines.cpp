#include <Magick++.h>
#include <iostream>
#include <fstream>

#include "libroad/hwm_network.hpp"
#include "im_heightfield.hpp"

int main(int argc, char *argv[])
{
    std::cerr << libroad_package_string() << std::endl;

    if(argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " <input image>" << std::endl;
        return 1;
    }

    Magick::Image im(argv[1]);

    im.quantizeColorSpace(Magick::GRAYColorspace);
    im.quantize();

    im.flip();

    vec2i dim(im.columns(), im.rows());

    float *pix = new float[dim[0]*dim[1]];
    im.write(0, 0, dim[1], dim[0], "R", Magick::FloatPixel, pix);

    im_heightfield ih(pix, dim, 0, 24);

    ih.origin = vec2f(-50.0);
    ih.spacing = vec2f(1);

    std::vector<vertex> vrts;
    std::vector<vec3u>  fcs;
    ih.make_mesh(vrts, fcs);
    std::ofstream os("im.obj");
    mesh_to_obj(os, "test", "test", vrts, fcs);

    std::vector<vec3f> poly;
    poly.push_back(vec3f(0.0));
    poly.push_back(vec3f(0.0, 5.0, 0.0));
    poly.push_back(vec3f(10.0));

    poly = ih.displace_polyline(poly, 0.1, 0.01);

    delete[] pix;

    vrts.clear();
    fcs.clear();
    arc_road ar;
    ar.initialize_from_polyline(0.0f, poly);
    ar.make_mesh(vrts, fcs, vec2f(0, 1), vec2f(-0.1, 0.1), 0.01);

    mesh_to_obj(os, "poly", "poly", vrts, fcs);

    return 0;
}
