#include "libroad/partition01.hpp"
#include "libroad/arc_road.hpp"

#include <fstream>

struct road
{
    typedef partition01<vec2u> lane_intervals;

    lane_intervals lanes;
    arc_road       rep;
};

static const float lane_width     = 2.5;
static const float shoulder_width = 3.5;
enum { SHOULDER = 0, LANE = 1, CENTER = 2 };
const char *mtl_string(int s)
{
    static const char* strs[3] = { "shoulder",
                                   "lane",
                                   "center" };

    if(s < 0 || s > 2)
        return "unknown";

    return strs[s];
}

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

struct edge
{
    edge() {};
    edge(const int v0, const vec2f &tc0,
         const int v1, const vec2f &tc1,
         const int mat)
        : mat_id(mat)
    {
        vrts[0] = v0;
        vrts[1] = v1;
        tc[0]   = tc0;
        tc[1]   = tc1;
    }

    int   vrts[2];
    vec2f tc[2];
    int   mat_id;
};

struct edge_ring
{
    void transform(const mat4x4f &mat)
    {
        BOOST_FOREACH(vec3f &v, verts)
        {
            const vec4f v4i(v[0], v[1], v[2], 1.0);
            const vec4f v4o(mat * v4i);
            for(int i = 0; i < 3; ++i)
                v[i] = v4o[i];
        }
    }

    std::vector<vec3f> verts;
    std::vector<edge>  edges;
};

struct mesh
{
    std::vector<vec3f>   verts;
    std::vector<polygon> faces;

    void dump_obj(std::ostream &out)
    {
        BOOST_FOREACH(const vec3f &v, verts)
        {
            out << "v " << v[0] << " " << v[1] << " " << v[2] << "\n";
        }

        BOOST_FOREACH(const polygon &f, faces)
        {
            for(int i = 0; i < f.N; ++i)
                out << "vt " << f.tc[i][0] << " " << f.tc[i][1] << "\n";

            out << "usemtl " << mtl_string(f.mat_id) << "\nf ";
            for(int i = 0; i < f.N; ++i)
            {
                out << f.vrts[i]+1 << "/" << i-f.N << " ";
            }
            out << "\n";
        }
    }
};


mesh stitch_edge_rings(const std::vector<edge_ring> &er)
{
    mesh res;

    const edge_ring *last = &(er.front());
    size_t last_v_base = res.verts.size();
    res.verts.insert(res.verts.end(), last->verts.begin(), last->verts.end());

    for(std::vector<edge_ring>::const_iterator current = boost::next(er.begin());
        current != er.end(); ++current)
    {
        size_t current_v_base      = res.verts.size();
        res.verts.insert(res.verts.end(), current->verts.begin(), current->verts.end());
        assert(last->verts.size() == current->verts.size());
        assert(last->edges.size() == current->edges.size());

        std::vector<edge>::const_iterator ledge = last->edges.begin();
        std::vector<edge>::const_iterator cedge = current->edges.begin();
        while(ledge != last->edges.end() && cedge != current->edges.end())
        {
            const float t0 = distance(res.verts[ledge->vrts[0]+last_v_base],
                                      res.verts[cedge->vrts[0]+current_v_base]);
            const float t1 = distance(res.verts[ledge->vrts[1]+last_v_base],
                                      res.verts[cedge->vrts[1]+current_v_base]);
            assert(ledge->mat_id == cedge->mat_id);
            res.faces.push_back(polygon(ledge->vrts[1]+last_v_base, ledge->tc[1],
                                        ledge->vrts[0]+last_v_base, ledge->tc[0],
                                        cedge->vrts[0]+current_v_base, vec2f(cedge->tc[0]+vec2f(t0, 0.0f)),
                                        cedge->vrts[1]+current_v_base, vec2f(cedge->tc[1]+vec2f(t1, 0.0f)),
                                        ledge->mat_id));
            ++ledge;
            ++cedge;
        }

        last        = &(*current);
        last_v_base = current_v_base;
    }

    return res;
}

/*
  v shoulder v | v lane -1 v . v lane -0 v || ^ lane 0 ^ . ^ lane 1 ^ . ^ lane 2 ^ | ^ shoulder ^
                ^-lane width-^^-lane width-^
 */

edge_ring road_verts(const vec2i &lanes)
{
    edge_ring res;

    res.verts.push_back(vec3f(0.0f, -(lane_width*lanes[0] + shoulder_width), 0.0f));
    res.verts.push_back(vec3f(0.0f, -lane_width*lanes[0], 0.0f));
    res.edges.push_back(edge(0, vec2f(0.0f, 1.0f),
                             1, vec2f(0.0f, 0.0f),
                             SHOULDER));
    int n = 2;
    for(int i = lanes[0] - 1; i > 0; --i)
    {
        res.verts.push_back(vec3f(0.0f, -lane_width*i, 0.0f));
        n++;
        res.edges.push_back(edge(n-2, vec2f(0.0f, 1.0f),
                                 n-1, vec2f(0.0f, 0.0f),
                                 LANE));
    }
    res.verts.push_back(vec3f(0.0f, 0.0f, 0.0f));
    n++;

    if(lanes[1] > 0)
    {
        res.edges.push_back(edge(n-2, vec2f(0.0f, 1.0f),
                                 n-1, vec2f(0.0f, 0.0f),
                                 CENTER));

        res.verts.push_back(vec3f(0.0f, lane_width, 0.0f));
        n++;
        res.edges.push_back(edge(n-2, vec2f(0.0f, 0.0f),
                                 n-1, vec2f(0.0f, 1.0f),
                                 CENTER));

        for(int i = 2; i < lanes[1]; ++i)
        {
            res.verts.push_back(vec3f(0.0f, lane_width*i, 0.0f));
            n++;
            res.edges.push_back(edge(n-2, vec2f(0.0f, 0.0f),
                                     n-1, vec2f(0.0f, 1.0f),
                                     LANE));
        }
    }

    res.verts.push_back(vec3f(0.0f, lane_width*lanes[1] + shoulder_width, 0.0f));
    n++;
    res.edges.push_back(edge(n-2, vec2f(0.0f, 0.0f),
                             n-1, vec2f(0.0f, 1.0f),
                             SHOULDER));


    return res;
}

int main(int argc, char *argv[])
{
    std::cerr << libroad_package_string() << std::endl;

    std::vector<edge_ring> erv;
    erv.push_back(road_verts(vec2i(1, 1)));
    // erv.back().transform();

    erv.push_back(road_verts(vec2i(1, 1)));
    mat4x4f v;
    v = 1.0f, 0.0f, 0.0f, 10.0f,
        0.0f, 1.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f;
    erv.back().transform(v);

    erv.push_back(road_verts(vec2i(1, 1)));
    v = 0.0f, -1.0f, 0.0f, 20.0f,
        1.0f, 0.0f, 0.0f, 20.0f,
        0.0f, 0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f;
    erv.back().transform(v);

    mesh me(stitch_edge_rings(erv));

    me.dump_obj(std::cout);

    return 0;
}
