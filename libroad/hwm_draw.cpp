#include "hwm_draw.hpp"

static inline tvmet::XprVector<tvmet::VectorConstReference<float, 3>, 3> cvec3f(const float *mem)
{
    return tvmet::cvector_ref<float,3>(mem);
}

namespace hwm
{
    car_draw::car_draw() : v_vbo(0), n_vbo(0)
    {}

    bool car_draw::initialized() const
    {
        return v_vbo && n_vbo;
    }

    void car_draw::initialize(const float car_width,
                              const float car_length,
                              const float car_height,
                              const float car_rear_axle)
    {
        const float overts[][3] = {{-(car_length-car_rear_axle), -car_width/2, 0.0f},  //0
                                   {              car_rear_axle, -car_width/4, 0.0f},  //1
                                   {              car_rear_axle,  car_width/4, 0.0f},  //2
                                   {-(car_length-car_rear_axle),  car_width/2, 0.0f},  //3

                                   {-(car_length-car_rear_axle), -car_width/2,       car_height},  //4
                                   {              car_rear_axle, -car_width/4, car_height*13/15},  //5
                                   {              car_rear_axle,            0, car_height*13/15},  //6
                                   {-(car_length-car_rear_axle),            0,      car_height}};  //7

        const unsigned int ofaces[6][4] = {{ 7, 6, 5, 4}, // bottom
                                           { 0, 1, 2, 3}, // top
                                           { 1, 5, 4, 0}, // left side
                                           { 0, 3, 7, 4}, // back
                                           { 3, 7, 6, 2}, // right side
                                           { 5, 6, 2, 1}};// front

        std::vector<vec3f>        verts  (24);
        std::vector<vec3f>        normals(24);

        for(int i = 0; i < 6; ++i)
        {
            const vec3f d10 (cvec3f(overts[ofaces[i][1]]) - cvec3f(overts[ofaces[i][0]]));
            const vec3f d20 (cvec3f(overts[ofaces[i][2]]) - cvec3f(overts[ofaces[i][0]]));
            vec3f norm(tvmet::normalize(tvmet::cross(d10, d20)));

            for(int j = 0; j < 4; ++j)
            {
                verts[i*4+j]   = cvec3f(overts[ofaces[i][j]]);
                normals[i*4+j] = norm;
            }
        }

        glGenBuffersARB(1, &v_vbo);
        glBindBufferARB(GL_ARRAY_BUFFER_ARB, v_vbo);
        glBufferDataARB(GL_ARRAY_BUFFER_ARB, 24*3*sizeof(float), &(verts[0]), GL_STATIC_DRAW_ARB);

        glGenBuffersARB(1, &n_vbo);
        glBindBufferARB(GL_ARRAY_BUFFER_ARB, n_vbo);
        glBufferDataARB(GL_ARRAY_BUFFER_ARB, 24*3*sizeof(float), &(normals[0]), GL_STATIC_DRAW_ARB);
        assert(glGetError() == GL_NO_ERROR);
    }

    void car_draw::draw() const
    {
        glBindBufferARB(GL_ARRAY_BUFFER_ARB, v_vbo);
        glEnableClientState(GL_VERTEX_ARRAY);
        glVertexPointer(3, GL_FLOAT, 0, 0);

        glBindBufferARB(GL_ARRAY_BUFFER_ARB, n_vbo);
        glEnableClientState(GL_NORMAL_ARRAY);
        glNormalPointer(GL_FLOAT, 0, 0);

        assert(glGetError() == GL_NO_ERROR);
        glDrawArrays(GL_QUADS, 0, 24);

        glDisableClientState(GL_VERTEX_ARRAY);
        glDisableClientState(GL_NORMAL_ARRAY);
        assert(glGetError() == GL_NO_ERROR);
    }

    car_draw::~car_draw()
    {
        if(v_vbo)
            glDeleteBuffersARB(1, &v_vbo);
        if(n_vbo)
            glDeleteBuffersARB(1, &n_vbo);
    }

    network_draw::network_draw() : v_vbo(0), f_vbo(0)
    {}

    bool network_draw::initialized() const
    {
        return v_vbo && f_vbo;
    }

    void network_draw::initialize(const network *net, const float lane_width)
    {
        std::vector<vertex> lane_points;
        std::vector<vec3u>  lane_faces;

        typedef std::pair<const str, hwm::lane> lmap_itr;
        BOOST_FOREACH(const lmap_itr &l, net->lanes)
        {
            lane_vert_starts.push_back(lane_points.size());
            lane_face_starts.push_back(lane_faces.size());

            l.second.make_mesh(lane_points, lane_faces, lane_width, 2.0f);

            lane_vert_counts.push_back(lane_points.size()-lane_vert_starts.back());
            lane_face_counts.push_back(lane_faces.size() -lane_face_starts.back());
        }
        BOOST_FOREACH(GLsizei &i, lane_face_counts)
        {
            i *= 3;
        }
        BOOST_FOREACH(size_t &i, lane_face_starts)
        {
            i *= sizeof(vec3u);
        }

        glGenBuffersARB(1, &v_vbo);
        glBindBufferARB(GL_ARRAY_BUFFER_ARB, v_vbo);
        glBufferDataARB(GL_ARRAY_BUFFER_ARB, lane_points.size()*sizeof(vertex), &(lane_points[0]), GL_STATIC_DRAW_ARB);

        glGenBuffersARB(1, &f_vbo);
        glBindBufferARB(GL_ELEMENT_ARRAY_BUFFER_ARB, f_vbo);
        glBufferDataARB(GL_ELEMENT_ARRAY_BUFFER_ARB, lane_faces.size()*sizeof(vec3i), &(lane_faces[0]), GL_STATIC_DRAW_ARB);

        assert(glGetError() == GL_NO_ERROR);
    }

    void network_draw::draw_wire()
    {
        glBindBufferARB(GL_ARRAY_BUFFER_ARB, v_vbo);
        glEnableClientState(GL_VERTEX_ARRAY);
        glVertexPointer(3, GL_FLOAT, sizeof(vertex), 0);

        assert(glGetError() == GL_NO_ERROR);
        glMultiDrawArrays(GL_LINE_LOOP, &(lane_vert_starts[0]), &(lane_vert_counts[0]), lane_vert_starts.size());

        glDisableClientState(GL_VERTEX_ARRAY);
        assert(glGetError() == GL_NO_ERROR);
    }

    void network_draw::draw_solid()
    {
        glBindBufferARB(GL_ARRAY_BUFFER_ARB, v_vbo);
        glEnableClientState(GL_VERTEX_ARRAY);
        glVertexPointer(3, GL_FLOAT, sizeof(vertex), 0);

        glEnableClientState(GL_NORMAL_ARRAY);
        glNormalPointer(GL_FLOAT, sizeof(vertex), reinterpret_cast<void*>(sizeof(vec3f)));

        glBindBufferARB(GL_ELEMENT_ARRAY_BUFFER_ARB, f_vbo);

        assert(glGetError() == GL_NO_ERROR);
        glMultiDrawElements(GL_TRIANGLES, &(lane_face_counts[0]), GL_UNSIGNED_INT, reinterpret_cast<const GLvoid**>(&(lane_face_starts[0])), lane_face_starts.size());

        glDisableClientState(GL_VERTEX_ARRAY);
        glDisableClientState(GL_NORMAL_ARRAY);
        assert(glGetError() == GL_NO_ERROR);

        glBindBufferARB(GL_ARRAY_BUFFER_ARB, 0);
        glBindBufferARB(GL_ELEMENT_ARRAY_BUFFER_ARB, 0);
    }


    network_draw::~network_draw()
    {
        if(v_vbo)
            glDeleteBuffersARB(1, &v_vbo);
        if(f_vbo)
            glDeleteBuffersARB(1, &f_vbo);
    }
}
