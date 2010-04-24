#ifndef __IM_HEIGHTFIELD_HPP__
#define __IM_HEIGHTFIELD_HPP__

#include "libroad/hwm_network.hpp"
#include "libroad/osm_network.hpp"

struct im_heightfield
{
    im_heightfield(const float *p, const vec2i &d, float zb, float zs)
        : pix(p), dim(d), zbase(zb), zscale(zs)
    {}

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

    vec3f vlookup(const vec2f &xy) const
    {
        return vec3f(xy[0], xy[1], lookup(xy));
    }

    vec3f vlookup(const vec3f &xy) const
    {
        return vec3f(xy[0], xy[1], xy[2] + lookup(vec2f(xy[0], xy[1])));
    }

    std::vector<vec3f> displace_polyline(const std::vector<vec3f> &poly, const float fac_limit) const
    {
        std::vector<vec3f>                 res;
        std::vector<vec3f>                 to_add;
        std::vector<vec3f>::const_iterator input(poly.begin());

        res.push_back(vlookup(*input++));
        to_add.push_back(vlookup(*input++));

        vec3f split;
        while(true)
        {
            while(!to_add.empty())
            {
                while(max_diff(split, res.back(), to_add.back(), fac_limit))
                {
                    to_add.push_back(split);
                }
                res.push_back(to_add.back());
                to_add.pop_back();
            }

            if(input == poly.end())
                return res;

            to_add.push_back(vlookup(*input++));
        }
    }

    void displace_shapes(osm::shape_t &s, const float fac_limit, osm::network &net) const
    {
        osm::shape_t                      res;
        osm::shape_t           to_add;
        osm::shape_t::iterator input(s.begin());

        (*input)->xy = vlookup((*input)->xy);
        res.push_back(*input);
        ++input;

        (*input)->xy = vlookup((*input)->xy);
        to_add.push_back(*input);
        ++input;

        vec3f split;
        while(true)
        {
            while(!to_add.empty())
            {
                while(max_diff(split, res.back()->xy, to_add.back()->xy, fac_limit))
                {
                    to_add.push_back(net.add_node(split, res.back()->is_overpass && to_add.back()->is_overpass));
                }

                res.push_back(to_add.back());
                to_add.pop_back();
            }

            if(input == s.end())
                s.swap(res);

            (*input)->xy = vlookup((*input)->xy);
            to_add.push_back(*input);
            ++input;
        }
    }

    bool max_diff(vec3f &best, const vec3f &v0, const vec3f &v1, const float fac_limit) const
    {
        const vec3f dvec(v1-v0);
        const float len(length(dvec));
        const float inv_len = 1.0f/len;
        const vec3f nvec(dvec*inv_len);

        const float dt       = 0.5*std::min(spacing[0], spacing[1])*inv_len;
        float       best_fac = fac_limit;
        bool        best_set = false;
        for(float t = dt; t < len; t += dt)
        {
            const vec3f mid(v0 + nvec*t);
            const vec3f cand(vlookup(mid));
            float       cand_fac(std::abs(cand[2] - mid[2])*inv_len);
            if(cand_fac > best_fac)
            {
                best_set = true;
                best     = cand;
                best_fac = cand_fac;
            }
        }
        return best_set;
    };

    void make_mesh(std::vector<vertex> &vrts, std::vector<vec3u> &faces) const
    {
        for(int j = 0; j < dim[1]; ++j)
        {
            for(int i = 0; i < dim[0]; ++i)
            {
                const vec3f pos(i*spacing[0] + origin[0],
                                j*spacing[1] + origin[1],
                                zbase + zscale*pix[i + j * dim[0]]);

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

    const float *pix;
    vec2i        dim;

    float zbase;
    float zscale;

    vec2f origin;
    vec2f spacing;
};

#endif
