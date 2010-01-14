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

    network_draw::network_draw() : v_vbo(0), f_vbo(0), net(0)
    {}

    bool network_draw::initialized() const
    {
        return net && v_vbo && f_vbo;
    }

    void network_draw::initialize(const network *in_net, const float lane_width)
    {
        net = in_net;
        std::vector<vertex> points;
        std::vector<vec3u>  lane_faces;

        typedef std::pair<const str, lane> lmap_itr;
        BOOST_FOREACH(const lmap_itr &l, net->lanes)
        {
            lane_vert_starts.push_back(points.size());
            lane_face_starts.push_back(lane_faces.size());

            l.second.make_mesh(points, lane_faces, lane_width, 2.0f);

            lane_vert_counts.push_back(points.size()-lane_vert_starts.back());
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

        typedef std::pair<const str, intersection> imap_itr;
        BOOST_FOREACH(const imap_itr &i, net->intersections)
        {
            intersection_vert_fan_starts.push_back(points.size());

            points.push_back(std::make_pair(i.second.center, vec3f(0.0, 0.0, 1.0)));
            BOOST_FOREACH(const vec3f &p, i.second.shape)
            {
                points.push_back(std::make_pair(p, vec3f(0.0, 0.0, 1.0)));
            }
            points.push_back(std::make_pair(i.second.shape.front(), vec3f(0.0, 0.0, 1.0)));

            intersection_vert_fan_counts.push_back(points.size()-intersection_vert_fan_starts.back());

            intersection_vert_loop_starts.push_back(intersection_vert_fan_starts.back() + 1);
            intersection_vert_loop_counts.push_back(intersection_vert_fan_counts.back() - 2);

            BOOST_FOREACH(const intersection::state &st, i.second.states)
            {
                BOOST_FOREACH(const intersection::state::state_pair &sp, st.in_pair())
                {
                    assert(sp.fict_lane);
                    const lane &fict_lane = *(sp.fict_lane);

                    strhash<fict_lane_data>::type::iterator it = fictitious_lanes.find(fict_lane.id);
                    assert(it == fictitious_lanes.end());

                    fict_lane_data fld;
                    fld.vert_start = points.size();
                    fld.face_start = lane_faces.size();
                    fict_lane.make_mesh(points, lane_faces, lane_width, 2.0f);
                    fld.vert_count = points.size()     - fld.vert_start;
                    fld.face_count = (lane_faces.size() - fld.face_start) * 3;
                    fld.face_start *= sizeof(vec3u);

                    fictitious_lanes.insert(it, std::make_pair(fict_lane.id, fld));
                }
            }
        }

        glGenBuffersARB(1, &v_vbo);
        glBindBufferARB(GL_ARRAY_BUFFER_ARB, v_vbo);
        glBufferDataARB(GL_ARRAY_BUFFER_ARB, points.size()*sizeof(vertex), &(points[0]), GL_STATIC_DRAW_ARB);

        glGenBuffersARB(1, &f_vbo);
        glBindBufferARB(GL_ELEMENT_ARRAY_BUFFER_ARB, f_vbo);
        glBufferDataARB(GL_ELEMENT_ARRAY_BUFFER_ARB, lane_faces.size()*sizeof(vec3i), &(lane_faces[0]), GL_STATIC_DRAW_ARB);

        assert(glGetError() == GL_NO_ERROR);
    }

    void network_draw::draw_lanes_wire()
    {
        glBindBufferARB(GL_ARRAY_BUFFER_ARB, v_vbo);
        glEnableClientState(GL_VERTEX_ARRAY);
        glVertexPointer(3, GL_FLOAT, sizeof(vertex), 0);

        assert(glGetError() == GL_NO_ERROR);
        glMultiDrawArrays(GL_LINE_LOOP, &(lane_vert_starts[0]), &(lane_vert_counts[0]), lane_vert_starts.size());

        glDisableClientState(GL_VERTEX_ARRAY);
        assert(glGetError() == GL_NO_ERROR);
    }

    void network_draw::draw_lanes_solid()
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

    void network_draw::draw_intersections_wire()
    {
        glBindBufferARB(GL_ARRAY_BUFFER_ARB, v_vbo);
        glEnableClientState(GL_VERTEX_ARRAY);
        glVertexPointer(3, GL_FLOAT, sizeof(vertex), 0);

        assert(glGetError() == GL_NO_ERROR);
        glMultiDrawArrays(GL_LINE_LOOP, &(intersection_vert_loop_starts[0]), &(intersection_vert_loop_counts[0]), intersection_vert_loop_starts.size());

        glDisableClientState(GL_VERTEX_ARRAY);
        assert(glGetError() == GL_NO_ERROR);
    }

    void network_draw::draw_intersections_solid()
    {
        glBindBufferARB(GL_ARRAY_BUFFER_ARB, v_vbo);
        glEnableClientState(GL_VERTEX_ARRAY);
        glVertexPointer(3, GL_FLOAT, sizeof(vertex), 0);

        glEnableClientState(GL_NORMAL_ARRAY);
        glNormalPointer(GL_FLOAT, sizeof(vertex), reinterpret_cast<void*>(sizeof(vec3f)));

        assert(glGetError() == GL_NO_ERROR);
        glMultiDrawArrays(GL_TRIANGLE_FAN, &(intersection_vert_fan_starts[0]), &(intersection_vert_fan_counts[0]), intersection_vert_fan_starts.size());

        glDisableClientState(GL_VERTEX_ARRAY);
        glDisableClientState(GL_NORMAL_ARRAY);
        assert(glGetError() == GL_NO_ERROR);

        glBindBufferARB(GL_ARRAY_BUFFER_ARB, 0);
    }

    void network_draw::draw_fictitious_lanes_wire()
    {
        glBindBufferARB(GL_ARRAY_BUFFER_ARB, v_vbo);
        glEnableClientState(GL_VERTEX_ARRAY);
        glVertexPointer(3, GL_FLOAT, sizeof(vertex), 0);

        assert(glGetError() == GL_NO_ERROR);
        typedef std::pair<const str, intersection> imap_itr;
        BOOST_FOREACH(const imap_itr &i, net->intersections)
        {
            const intersection::state &st = i.second.states[i.second.current_state];
            BOOST_FOREACH(const intersection::state::state_pair &sp, st.in_pair())
            {
                assert(sp.fict_lane);
                const lane &fict_lane = *(sp.fict_lane);

                strhash<fict_lane_data>::type::iterator it = fictitious_lanes.find(fict_lane.id);
                assert(it != fictitious_lanes.end());

                const fict_lane_data &fld = it->second;
                glDrawArrays(GL_LINE_LOOP, fld.vert_start, fld.vert_count);
            }
        }

        glDisableClientState(GL_VERTEX_ARRAY);
        assert(glGetError() == GL_NO_ERROR);
    }

    void network_draw::draw_fictitious_lanes_solid()
    {
        glBindBufferARB(GL_ARRAY_BUFFER_ARB, v_vbo);
        glEnableClientState(GL_VERTEX_ARRAY);
        glVertexPointer(3, GL_FLOAT, sizeof(vertex), 0);

        glEnableClientState(GL_NORMAL_ARRAY);
        glNormalPointer(GL_FLOAT, sizeof(vertex), reinterpret_cast<void*>(sizeof(vec3f)));

        glBindBufferARB(GL_ELEMENT_ARRAY_BUFFER_ARB, f_vbo);

        assert(glGetError() == GL_NO_ERROR);
        typedef std::pair<const str, intersection> imap_itr;
        BOOST_FOREACH(const imap_itr &i, net->intersections)
        {
            const intersection::state &st = i.second.states[i.second.current_state];
            BOOST_FOREACH(const intersection::state::state_pair &sp, st.in_pair())
            {
                assert(sp.fict_lane);
                const lane &fict_lane = *(sp.fict_lane);

                strhash<fict_lane_data>::type::iterator it = fictitious_lanes.find(fict_lane.id);
                assert(it != fictitious_lanes.end());

                const fict_lane_data &fld = it->second;
                assert(glGetError() == GL_NO_ERROR);
                glDrawElements(GL_TRIANGLES, fld.face_count, GL_UNSIGNED_INT, reinterpret_cast<const GLvoid*>(fld.face_start));
            }
        }

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
