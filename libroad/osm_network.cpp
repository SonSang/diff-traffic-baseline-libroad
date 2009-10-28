#include "osm_network.hpp"
#include <FL/gl.h>
#include <FL/glu.h>
#include <FL/glut.H>

vec2d bias, prev;
bool first = 1;
double scale = 20.0;
namespace osm
{
    bool network::draw_network()
    {
        glColor3f(0,0,0);

        typedef std::pair<const str, edge> emap_pair;
        BOOST_FOREACH(emap_pair &ep, edges)
        {
            edge& e = ep.second;
            shape_t& e_nodes = e.shape;
            glBegin(GL_LINES);
            BOOST_FOREACH(vec2d& pt, e_nodes)
            {
                if (first)
                {
                    first = 0;
                    bias = pt;
                }
                glVertex3f((pt[0] - bias[0])*scale, (pt[1] - bias[1])*scale, 0);

                if ((pt[0] != e_nodes[0][0]) and
                          (pt[1] != e_nodes[0][1]))
                {
                    glVertex3f((prev[0] - bias[0])*scale, (prev[1] - bias[1])*scale, 0);
                }

                prev = pt;
            }
            glEnd();
        }

    }

    bool network::compute_edge_types()
    {
        std::cout << "compute_edge_types\n";

        typedef std::pair<const str, edge> emap_pair;
        BOOST_FOREACH( emap_pair &ep, edges)
        {
            edge& e = ep.second;
            edge_type* e_type = retrieve<edge_type>(types, ep.first);
            e.type = e_type;
            e_type->speed = 25;
            e_type->nolanes = 1;
            e_type->oneway = 0;

            std::cout << e.highway_class << "\n";
            if(e.highway_class == "residential")
            {
                e_type->speed = 30;
                e_type->nolanes = 1;
                e_type->oneway = 0;
            }
            if(e.highway_class == "primary")
            {
                e_type->nolanes = 2;
                e_type->oneway = 0;
                e_type->speed = 50;
            }
            if(e.highway_class == "secondary")
            {
                e_type->speed = 40;
                e_type->nolanes = 2;
                e_type->oneway = 0;
            }
            if(e.highway_class == "service")
            {
                e_type->speed = 25;
                e_type->nolanes = 1;
                e_type->oneway = 0;
            }
            if(e.highway_class == "primary_link")
            {
                e_type->speed = 30;
                e_type->nolanes = 1;
                e_type->oneway = 1;
            }
            if(e.highway_class == "secondary_link")
            {
                e_type->speed = 30;
            e_type->nolanes = 1;
            e_type->oneway = 1;
            }
            std::cout << "speed " << e_type->speed << "\n";

        }
    }
}
