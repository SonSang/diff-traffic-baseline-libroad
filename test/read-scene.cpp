#include "libroad/hwm_network.hpp"
#include "libroad/im_heightfield.hpp"
#include <Magick++.h>

int main(int argc, char *argv[])
{
    std::cout << libroad_package_string() << std::endl;
    if(argc != 9)
    {
        std::cerr << "Usage: " << argv[0] << " <input scene> <heightfield> <spacing> <originx> <originy> <zbase> <zscale> <output scene>" << std::endl;
        return 1;
    }

    std::vector<obj_record> objs(xml_read_scene(argv[1]));

    // Magick::Image im(argv[2]);
    // const vec2i dim(im.columns(), im.rows());
    // float *pix = new float[dim[0]*dim[1]];
    // im.write(0, 0, dim[1], dim[0], "R", Magick::FloatPixel, pix);

    // im_heightfield ih(pix, dim, boost::lexical_cast<float>(argv[6]), boost::lexical_cast<float>(argv[7]));
    // ih.origin  = vec2f(boost::lexical_cast<float>(argv[4]), boost::lexical_cast<float>(argv[5]));
    // ih.spacing = vec2f(boost::lexical_cast<float>(argv[3]), boost::lexical_cast<float>(argv[3]));

    // delete[] pix;

    // BOOST_FOREACH(obj_record &obj_r, objs)
    // {
    //     float dz = ih.project_bb(obj_r.box);
    //     obj_r.matrix(2, 3) -= dz;
    // }

    xml_write_scene(argv[8], objs);

    return 0;
}
