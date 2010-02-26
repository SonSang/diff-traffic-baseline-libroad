#include "osm_network.hpp"
#include <FL/gl.h>
#include <FL/glu.h>
#include <FL/glut.H>
#include <boost/fusion/sequence.hpp>
#include <boost/fusion/include/sequence.hpp>
#include <boost/fusion/container/vector.hpp>
#include <boost/fusion/include/vector.hpp>
#include <boost/fusion/container/vector/vector_fwd.hpp>
#include <boost/fusion/include/vector_fwd.hpp>
#include <boost/fusion/container/list.hpp>
#include <boost/fusion/include/list.hpp>
#include <boost/fusion/container/list/list_fwd.hpp>
#include <boost/fusion/include/list_fwd.hpp>

bool   first_for_display = 1;

using boost::fusion::at_c;

std::vector<boost::fusion::vector<double, double, double> > colors;

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
                colors.push_back(boost::fusion::vector<double, double, double>(rand()/(double)RAND_MAX,rand()/(double)RAND_MAX,rand()/(double)RAND_MAX));
            }
        }

        BOOST_FOREACH(edge &e, edges)
        {
            const shape_t& e_nodes = e.shape;

            glBegin(GL_LINE_STRIP);
            glColor3f(at_c<0>(colors[i]),
                      at_c<1>(colors[i]),
                      at_c<2>(colors[i]));

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
