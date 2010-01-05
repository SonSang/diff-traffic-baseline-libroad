#include "libroad/arc_road.hpp"

int main(int argc, char *argv[])
{
    polyline_road pr;
    pr.points_.push_back(vec3f(0.0, 4.0, 0.0));
    pr.points_.push_back(vec3f(4.0, 3.0, 0.0));
    pr.points_.push_back(vec3f(4.0, 0.0, 0.0));
    pr.points_.push_back(vec3f(6.0, 0.0, 0.0));
    pr.points_.push_back(vec3f(3.0, -2.0, 0.0));
    pr.points_.push_back(vec3f(2.0, -1.0, 0.0));
    pr.points_.push_back(vec3f(2.0, -4.0, 0.0));
    pr.points_.push_back(vec3f(0.5, -2.0, 0.0));
    pr.points_.push_back(vec3f(0.5, -5.0, 0.0));
    pr.points_.push_back(vec3f(1.0, -7.0, 0.0));
    pr.points_.push_back(vec3f(0.0, -9, 0.0));
    pr.points_.push_back(vec3f(5.0, -8, 0.0));
    pr.points_.push_back(vec3f(7.5, -5, 0.0));
    pr.points_.push_back(vec3f(10, -3, 0.0));
    pr.initialize();

    arc_road ar(pr);

    // for(size_t i = 0; i < ar.frames_.size(); ++i)
    // {
    //     std::cout << "point  " << i << std::endl
    //               << "radius " << ar.radii_[i] << std::endl
    //               << "arc    " << ar.arcs_[i] << std::endl
    //               << ar.frames_[i] << std::endl << std::endl;
    // }

    std::vector<vec3f> res(ar.extract_line(0.1f, 2.0f));
    for(size_t i = 0; i < res.size(); ++i)
    {
        std::cout << i << " " << res[i] << std::endl;
    }

    return 0;
}
