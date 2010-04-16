#include "libroad/arc_road.hpp"

#include <fstream>

void dump_obj(std::ostream &out, const std::vector<vertex> &verts,
              std::vector<vec3u> &faces)
{
    BOOST_FOREACH(const vertex &v, verts)
    {
        out << "v " << v.position[0] << " " << v.position[1] << " " << v.position[2]  << "\n"
            << "vn " << v.normal[0] << " " << v.normal[1] << " " << v.normal[2]  << "\n"
            << "vt " << v.tex_coord[0] << " " << v.tex_coord[1] << "\n";
    }

    BOOST_FOREACH(const vec3u &f, faces)
    {
        out << "f ";
        BOOST_FOREACH(const unsigned int i, f)
        {
            out << i+1 << "/" << i+1 << "/" << i+1 << " ";
        }
        out << "\n";
    }
};

int main(int argc, char *argv[])
{
    std::cerr << libroad_package_string() << std::endl;

    arc_road ar;
    ar.points_.push_back(vec3f(000, 400, 0.0));
    ar.points_.push_back(vec3f(400, 300, 0.0));
    ar.points_.push_back(vec3f(400, 000, 0.0));
    ar.points_.push_back(vec3f(600, 000, 0.0));
    ar.points_.push_back(vec3f(300, -200, 0.0));
    ar.points_.push_back(vec3f(200, -100, 10));
    ar.points_.push_back(vec3f(200, -400, 10));
    ar.points_.push_back(vec3f(005, -200, 10));
    ar.points_.push_back(vec3f(005, -500, 20));
    ar.points_.push_back(vec3f(100, -700, 20));
    ar.points_.push_back(vec3f(000, -900, 20));
    ar.points_.push_back(vec3f(500, -800, 30));
    ar.points_.push_back(vec3f(705, -500, 30));
    ar.points_.push_back(vec3f(1000, -300, 30));
    ar.initialize_from_polyline(0.7f, ar.points_);

    std::vector<vertex> vrts;
    std::vector<vec3u>  fcs;

    ar.make_mesh(vrts, fcs, vec2f(0.0f, 1.0f), vec2f(-2*2.5f, 2*2.5f), 10.0f);

    dump_obj(std::cout, vrts, fcs);

    return 0;
}
