#include "osm_network.hpp"
#include <FL/gl.h>
#include <FL/glu.h>
#include <FL/glut.H>

bool   first_for_display = 1;

std::vector<vec3f> colors;

namespace osm
{
    void network::draw_network()
    {
        glColor3f(0,0,0);

        int i = 0;
        if (first_for_display)
        {
            for(size_t i = 0; i < edges.size(); ++i)
            {
                colors.push_back(vec3f(drand48(), drand48(), drand48()));
            }
        }

        BOOST_FOREACH(edge &e, edges)
        {
            const shape_t& e_nodes = e.shape;

            glBegin(GL_LINE_STRIP);
            glColor3fv(colors[i].data());

            for(int j = 0; j < static_cast<int>(e_nodes.size()); j++)
            {

                glVertex3f((e_nodes[j]->xy[0]),
                           (e_nodes[j]->xy[1]),
                           0);
            }
            glEnd();

            i++;
        }

        glPointSize(5);
        glColor3f(1,0,0);
        glBegin(GL_POINTS);
        BOOST_FOREACH(strhash<intersection>::type::value_type& i, intersections)
        {
            osm::node& n = nodes[i.second.id_from_node];
            glVertex3f(n.xy[0], n.xy[1], 0.0);
        }
        glEnd();

        first_for_display = false;
    }
}
