#include "libroad/partition01.hpp"
#include "libroad/arc_road.hpp"

struct road
{
    typedef partition01<vec2u> lane_intervals;

    lane_intervals lanes;
    arc_road       rep;
};

static const float lane_width     = 2.5;
static const float shoulder_width = 3.5;
//static const float line_width     = 0.0381;

struct polygon
{
    polygon() {};
    polygon(const int v0, const vec2f &tc0,
            const int v1, const vec2f &tc1,
            const int mat)
        : N(2), mat_id(mat)
    {
        vrts[0] = v0;
        vrts[1] = v1;
        tc[0]   = tc0;
        tc[1]   = tc1;
    }
    polygon(const int v0, const vec2f &tc0,
            const int v1, const vec2f &tc1,
            const int v2, const vec2f &tc2,
            const int mat)
        : N(3), mat_id(mat)
    {
        vrts[0] = v0;
        vrts[1] = v1;
        vrts[2] = v2;
        tc[0]   = tc0;
        tc[1]   = tc1;
        tc[2]   = tc2;
    }
    polygon(const int v0, const vec2f &tc0,
            const int v1, const vec2f &tc1,
            const int v2, const vec2f &tc2,
            const int v3, const vec2f &tc3,
            const int mat)
        : N(4), mat_id(mat)
    {
        vrts[0] = v0;
        vrts[1] = v1;
        vrts[2] = v2;
        vrts[3] = v3;
        tc[0]   = tc0;
        tc[1]   = tc1;
        tc[2]   = tc2;
        tc[3]   = tc3;
    }

    int   N;
    int   vrts[4];
    vec2f tc[4];
    int   mat_id;
};

struct mesh
{
    std::vector<vec3f>   verts;
    std::vector<polygon> faces;
};

/*
  v shoulder v | v lane -1 v . v lane -0 v || ^ lane 0 ^ . ^ lane 1 ^ . ^ lane 2 ^ | ^ shoulder ^
                ^-lane width-^^-lane width-^
 */

mesh road_verts(const vec2u &lanes)
{
    mesh res;

    int n = 0;
    res.verts.push_back(vec3f(0.0f, -(lane_width*lanes[0] + shoulder_width), 0.0f));
    n++;
    for(int i = lanes[0]; i > 0; --i)
    {
        res.verts.push_back(vec3f(0.0f, -lane_width*i, 0.0f));
        n++;
        res.faces.push_back(polygon(n-2, vec2f(0, 1),
                                    n-1, vec2f(0, 1),
                                    0));
    }
    for(int i = 0; i < lanes[1]+1; ++i)
        res[fill++]   = vec3f(0.0f, lane_width*i, 0.0f);
    res[fill++]       = vec3f(0.0f, lane_width*lanes[1] + shoulder_width, 0.0f);

    return res;
}

int main(int argc, char *argv[])
{
    std::cout << libroad_package_string() << std::endl;

    std::vector<vec3f> verts(road_verts(vec2u(1, 1)));

    BOOST_FOREACH(const vec3f &v, verts)
    {
        std::cout << v << " ";
    }
    std::cout << std::endl;

    return 0;
}
