#include <GL/glew.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include "hwm_network.hpp"

namespace hwm
{
    struct car_draw
    {
        car_draw();

        bool initialized() const;

        void initialize(const float car_width,
                        const float car_length,
                        const float car_height,
                        const float car_rear_axle);

        void draw() const;

        ~car_draw();

        GLuint v_vbo;
        GLuint n_vbo;
    };

    struct network_draw
    {
        network_draw();

        bool initialized() const;
        void initialize(const hwm::network *net, const float lane_width);

        void draw_lanes_wire();
        void draw_lanes_solid();

        void draw_intersections_wire();
        void draw_intersections_solid();

        ~network_draw();

        GLuint v_vbo;
        GLuint f_vbo;

        std::vector<GLint>    lane_vert_starts;
        std::vector<GLsizei>  lane_vert_counts;
        std::vector<size_t>   lane_face_starts;
        std::vector<GLsizei>  lane_face_counts;

        std::vector<GLint>    intersection_vert_fan_starts;
        std::vector<GLsizei>  intersection_vert_fan_counts;
        std::vector<GLint>    intersection_vert_loop_starts;
        std::vector<GLsizei>  intersection_vert_loop_counts;

    };
}
