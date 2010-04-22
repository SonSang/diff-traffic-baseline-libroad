#include "Magick++.h"
#include "libroad/hwm_network.hpp"
#include <iostream>

struct im_heightfield
{
    im_heightfield(Magick::Image &im, float zb, float zs)
        : zbase(zb), zscale(zs)
    {
        dim[0] = im.columns();
        dim[1] = im.rows();

        pix = new float[dim[0]*dim[1]];
        im.write(0, 0, dim[1], dim[0], "R", Magick::FloatPixel, pix);
    }

    ~im_heightfield()
    {
        delete[] pix;
    }

    float lookup(const vec2f &xy) const
    {
        const vec2f local((xy[0]-origin[0])/spacing[0],
                          (xy[1]-origin[1])/spacing[1]);

        vec2i base(static_cast<int>(std::floor(local[0])),
                   static_cast<int>(std::floor(local[1])));

        const vec2f remainder(local-base);

        if(base[0] < 0)        base[0] = 0;
        if(base[1] < 0)        base[1] = 0;
        if(base[0] > dim[0]-1) base[0] = dim[0]-1;
        if(base[1] > dim[1]-1) base[1] = dim[1]-1;

        const float sw(pix[base[0]     + base[1]*dim[0]]);
        const float se(pix[base[0] + 1 + base[1]*dim[0]]);
        const float ne(pix[base[0] + 1 + (base[1]+1)*dim[0]]);
        const float nw(pix[base[0]     + (base[1]+1)*dim[0]]);

        return zbase + zscale*((sw*(1-remainder[0]) + se*remainder[0])*(1-remainder[1]) +
                               (nw*(1-remainder[0]) + ne*remainder[0])*remainder[1]);
    }

    void make_mesh(std::vector<vertex> &vrts, std::vector<vec3u> &faces)
    {
        for(int j = 0; j < dim[1]; ++j)
        {
            for(int i = 0; i < dim[0]; ++i)
            {
                const vec3f pos(i*spacing[0] + origin[0],
                                j*spacing[1] + origin[1],
                                pix[i + j * dim[0]]);

                const vec2f uv(i/static_cast<float>(dim[0]-1),
                               j/static_cast<float>(dim[1]-1));

                vrts.push_back(vertex(pos, vec3f(0), uv));
            }
        }

        for(int j = 0; j < dim[1]; ++j)
        {
            for(int i = 0; i < dim[0]; ++i)
            {
                const vec3f &current(vrts[i+j*dim[0]].position);
                vec3f xback, yback, xnext, ynext;

                if(i > 0)        xback = vec3f(vrts[i-1+ j   *dim[0]].position - current);
                if(j > 0)        yback = vec3f(vrts[i  +(j-1)*dim[0]].position - current);
                if(i < dim[0]-1) xnext = vec3f(vrts[i+1+ j   *dim[0]].position - current);
                if(j < dim[1]-1) ynext = vec3f(vrts[i  +(j+1)*dim[0]].position - current);

                vrts[i+j*dim[1]].normal = tvmet::cross(0.5*(ynext-yback), 0.5*(xnext-xback));
                vrts[i+j*dim[1]].normal /= length(vrts[i+j*dim[0]].normal);
            }
        }

        for(int j = 1; j < dim[1]; ++j)
        {
            for(int i = 1; i < dim[0]; ++i)
            {
                faces.push_back(vec3u(i-1 + j*dim[0],
                                      i   + j*dim[0],
                                      i-1 + (j-1)*dim[0]));
                faces.push_back(vec3u(i   + j    *dim[0],
                                      i   + (j-1)*dim[0],
                                      i-1 + (j-1)*dim[0]));
            }
        }
    }

    float zbase;
    float zscale;

    vec2f origin;
    vec2f spacing;

    vec2i  dim;
    float *pix;
};

void dump_obj(std::ostream      &out,
              const std::string &name,
              const std::string &material_name,
              const std::vector<vertex> &verts,
              const std::vector<vec3u>  &faces)
{
    out << "o " << name << "\n";
    out << "usemtl " << material_name << "\n";
    BOOST_FOREACH(const vertex &v, verts)
    {
        out << "v " << v.position[0] << " " << v.position[1] << " " << v.position[2]  << "\n"
            << "vn " << v.normal[0] << " " << v.normal[1] << " " << v.normal[2]  << "\n"
            << "vt " << v.tex_coord[0] << " " << v.tex_coord[1] << "\n";
    }

    const size_t nverts = verts.size();
    BOOST_FOREACH(const vec3u &f, faces)
    {
        out << "f ";
        BOOST_FOREACH(const unsigned int i, f)
        {
            const off_t idx = static_cast<off_t>(i) - nverts;
            out << idx << "/" << idx << "/" << idx << " ";
        }
        out << "\n";
    }
};

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

    im_heightfield ih(im, 0, 1);

    ih.origin = vec2f(0.0);
    ih.spacing = vec2f(1.0/1023.0);

    std::vector<vertex> vrts;
    std::vector<vec3u>  fcs;

    ih.make_mesh(vrts, fcs);
    dump_obj(std::cout, "test", "test", vrts, fcs);

    return 0;
}
