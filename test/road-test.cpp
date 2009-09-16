#include "libroad/polyline_road.hpp"

#include <iostream>

int main(int argc, char *argv[])
{
    polyline_road *pr = new polyline_road();
    pr->points_.push_back(vec3f(0.0, 0.0, 0.0));
    pr->points_.push_back(vec3f(1.0, 0.0, 0.5));
    pr->points_.push_back(vec3f(1.0, 1.0, 0.5));
    pr->initialize();

    road_rep *rr = pr;

    std::cout << rr->length() << std::endl;
    vec3f pt;
    rr->point(0.5, pt);
    std::cout << pt << std::endl;

    mat3x3f mat3(0.0f);
    rr->frame(0.1, mat3);
    std::cout << mat3 << std::endl;

    mat4x4f mat4(0.0f);
    rr->point_frame(0.1, mat4);
    std::cout << mat4 << std::endl;
    //    std::cout << vec3f(tvmet::prod(mat, vec3f(1.0, 0.0, 0.0))) << std::endl;

    delete rr;

    return 0;
}