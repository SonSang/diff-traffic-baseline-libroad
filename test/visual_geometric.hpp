#ifndef _VISUAL_GEOMETRIC_HPP_
#define _VISUAL_GEOMETRIC_HPP_

#include "libroad/geometric.hpp"

inline void pick_ray(vec3f &origin, vec3f &dir, const float x, const float y, const int width, const int height, bool ortho = false)
{
    GLint view[4];
    glGetIntegerv(GL_VIEWPORT, view);
    GLdouble project[16];
    glGetDoublev(GL_PROJECTION_MATRIX, project);
    GLdouble model[16];
    glGetDoublev(GL_MODELVIEW_MATRIX, model);

    vec3d pt;
    vec3d o;

    GLdouble model2[16];
    GLdouble modelinv[16];

    for(int j = 0; j < 4; ++j)
        for(int i = 0; i < 4; ++i)
            model2[4*j+i] = model[4*i+j];

    memset(modelinv, 0, 16*sizeof(double));
    modelinv[4*0 + 0] = 1.0;
    modelinv[4*1 + 1] = 1.0;
    modelinv[4*2 + 2] = 1.0;
    modelinv[4*3 + 3] = 1.0;

    rm_invert(model2, modelinv);

    gluUnProject(x, height-y, 1.0, model, project, view, pt.data(), pt.data()+1, pt.data()+2);

    if(ortho)
        gluUnProject(x, height-y, -1.0, model, project, view, o.data(), o.data()+1, o.data()+2);
    else
    {
        o[0] = modelinv[4*0 + 3];
        o[1] = modelinv[4*1 + 3];
        o[2] = modelinv[4*2 + 3];
    }

    const vec3d v(tvmet::normalize(pt - o));
    for(int i = 0; i < 3; ++i)
    {
        origin[i] = o[i];
        dir[i]    = v[i];
    }
}
#endif
