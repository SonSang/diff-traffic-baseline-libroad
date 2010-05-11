#include "libroad/hwm_network.hpp"
#include "libroad/im_heightfield.hpp"
#include <Magick++.h>

int main(int argc, char *argv[])
{
    std::cout << libroad_package_string() << std::endl;
    if(argc != 11)
    {
        std::cerr << "Usage: " << argv[0] << " <input scene> <heightfield> <dimx> <dimy> <spacing> <originx> <originy> <zbase> <zscale> <output scene>" << std::endl;
        return 1;
    }

    std::vector<obj_record> objs(xml_read_scene(argv[1]));

    FILE *fp = fopen(argv[2], "rb");
    const vec2i dim(boost::lexical_cast<float>(argv[3]), boost::lexical_cast<float>(argv[4]));
    float *pix = new float[dim[0]*dim[1]];
    fread(pix, sizeof(float), dim[0]*dim[1], fp);
    im_heightfield ih(pix, dim, boost::lexical_cast<float>(argv[8]), boost::lexical_cast<float>(argv[9]));
    ih.origin  = vec2f(boost::lexical_cast<float>(argv[6]), boost::lexical_cast<float>(argv[7]));
    ih.spacing = vec2f(boost::lexical_cast<float>(argv[5]), boost::lexical_cast<float>(argv[5]));

    delete[] pix;

    BOOST_FOREACH(obj_record &obj_r, objs)
    {
        float dz = ih.project_bb(obj_r.box);
        obj_r.matrix(2, 3) -= dz;
    }

    xml_write_scene(argv[10], objs);

    return 0;
}
