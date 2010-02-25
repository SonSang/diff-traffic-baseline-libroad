#include "hwm_network.hpp"
#include "xml_util.hpp"

void polyline_road::xml_write(xmlpp::Element *elt) const
{
    xmlpp::Element *pr_elt = elt->add_child("line_rep");
    xmlpp::Element *pt_elt = pr_elt->add_child("points");

    BOOST_FOREACH(const vec3f &pt, points_)
    {
        pt_elt->add_child_text(boost::str(boost::format("%f %f %f 0.0\n") % pt[0] % pt[1] % pt[2]));
    }
}

void arc_road::xml_write_as_poly(xmlpp::Element *elt) const
{
    xmlpp::Element *ar_elt = elt->add_child("line_rep");
    xmlpp::Element *pt_elt = ar_elt->add_child("points");

    BOOST_FOREACH(const vec3f &pt, points_)
    {
        pt_elt->add_child_text(boost::str(boost::format("%f %f %f 0.0\n") % pt[0] % pt[1] % pt[2]));
    }
}

void arc_road::xml_write(xmlpp::Element *elt) const
{
    xmlpp::Element *ar_elt = elt->add_child("arc_line_rep");
    xmlpp::Element *pt_elt = ar_elt->add_child("points");

    BOOST_FOREACH(const vec3f &pt, points_)
    {
        pt_elt->add_child_text(boost::str(boost::format("%f %f %f 0.0\n") % pt[0] % pt[1] % pt[2]));
    }

    xmlpp::Element *r_elt = ar_elt->add_child("radii");

    BOOST_FOREACH(float pt, radii_)
    {
        r_elt->add_child_text(boost::str(boost::format("%f\n") % pt));
    }
}

void arc_road::svg_arc_arcs(const str &id, xmlpp::Element *parent) const
{
    if(points_.size() <= 2)
        return;

    xmlpp::Element *circle_group = parent->add_child("g");
    circle_group->set_attribute("id", boost::str(boost::format("id%s_arcs") % id));

    for(size_t i = 1; i < points_.size()-1; ++i)
    {
        xmlpp::Element *circle = circle_group->add_child("path");
        circle->set_attribute("id", boost::str(boost::format("id%s_arc_%d") % id % (i-1)));

        circle->set_attribute("d",  svg_arc_arc_path(i));
    }
}

void arc_road::svg_arc_circles(const str &id, xmlpp::Element *parent) const
{
    if(points_.size() <= 2)
        return;

    xmlpp::Element *circle_group = parent->add_child("g");
    circle_group->set_attribute("id", boost::str(boost::format("id%s_circles") % id));

    for(size_t i = 1; i < points_.size()-1; ++i)
    {
        const vec3f c(center(i));
        assert(std::isfinite(c[0]));
        xmlpp::Element *circle = circle_group->add_child("circle");
        circle->set_attribute("id", boost::str(boost::format("id%s_circle_%d") % id % (i-1)));

        circle->set_attribute("cx", boost::lexical_cast<str>(c[0]));
        circle->set_attribute("cy", boost::lexical_cast<str>(c[1]));
        circle->set_attribute("r", boost::lexical_cast<str>(radii_[i-1]));
    }
}

template <class T>
void partition01<T>::xml_write(xmlpp::Element *elt, const str &name) const
{
    xmlpp::Element *overall_elt = elt->add_child(name);
    xmlpp::Element *interval_elt = overall_elt->add_child("interval");

    typename partition01<T>::const_iterator pit = begin();
    if(!empty())
    {
        xmlpp::Element *base_elt = interval_elt->add_child("base");
        pit->second.xml_write(base_elt);
        for(++pit; pit != end(); ++pit)
        {
            xmlpp::Element *div_elt = interval_elt->add_child("divider");
            div_elt->set_attribute("value", boost::lexical_cast<str>(pit->first));
            pit->second.xml_write(div_elt);
        }
    }
}

namespace hwm
{
    template <class T>
    static inline void xml_write_map(const T &v, xmlpp::Element *elt, const str &name)
    {
        xmlpp::Element *map_elt = elt->add_child(name);

        typedef typename T::value_type val;
        BOOST_FOREACH(const val &item, v)
        {
            item.second.xml_write(map_elt);
        }
    }

    void road::xml_write(xmlpp::Element *elt) const
    {
        xmlpp::Element *road_elt = elt->add_child("road");
        road_elt->set_attribute("id",   id);
        road_elt->set_attribute("name", name);
        rep.xml_write(road_elt);
    }

    void lane::terminus::xml_write(xmlpp::Element *elt, const str &name) const
    {
        xmlpp::Element *term_elt = elt->add_child(name);
        term_elt->add_child("dead_end");
    }

    void lane::intersection_terminus::xml_write(xmlpp::Element *elt, const str &name) const
    {
        xmlpp::Element *term_elt = elt->add_child(name);
        xmlpp::Element *iref = term_elt->add_child("intersection_ref");
        iref->set_attribute("ref", adjacent_intersection->id);
    }

    void lane::lane_terminus::xml_write(xmlpp::Element *elt, const str &name) const
    {
        xmlpp::Element *term_elt = elt->add_child(name);
        xmlpp::Element *iref = term_elt->add_child("lane_ref");
        iref->set_attribute("ref", adjacent_lane->id);
    }

    void lane::road_membership::xml_write(xmlpp::Element *elt) const
    {
        xmlpp::Element *rm_elt = elt->add_child("road_membership");
        if(!empty())
        {
            rm_elt->set_attribute("parent_road_ref", parent_road->id);
            rm_elt->set_attribute("interval_start", boost::lexical_cast<str>(interval[0]));
            rm_elt->set_attribute("interval_end", boost::lexical_cast<str>(interval[1]));
            rm_elt->set_attribute("lane_position", boost::lexical_cast<str>(lane_position));
        }
    }

    void lane::adjacency::xml_write(xmlpp::Element *elt) const
    {
        xmlpp::Element *la_elt = elt->add_child("lane_adjacency");
        if(!empty())
        {
            la_elt->set_attribute("lane_ref",       neighbor->id);
            la_elt->set_attribute("interval_start", boost::lexical_cast<str>(neighbor_interval[0]));
            la_elt->set_attribute("interval_end",   boost::lexical_cast<str>(neighbor_interval[1]));
        }
    }

    void lane::xml_write(xmlpp::Element *elt) const
    {
        xmlpp::Element *lane_elt = elt->add_child("lane");
        lane_elt->set_attribute("id", id);
        lane_elt->set_attribute("speedlimit", boost::lexical_cast<str>(speedlimit));
        start->xml_write(lane_elt, "start");
        end->xml_write(lane_elt, "end");
        road_memberships.xml_write(lane_elt, "road_intervals");
        xmlpp::Element *adj_elt = lane_elt->add_child("adjacency_intervals");
        left. xml_write(adj_elt, "left");
        right.xml_write(adj_elt, "right");
    }

    template <class T>
    static inline void xml_write_vector(const std::vector<T> &v, xmlpp::Element *elt, const str &name)
    {
        xmlpp::Element *list_elt = elt->add_child(name);
        size_t count = 0;
        BOOST_FOREACH(const T &item, v)
        {
            xml_write(item, count, list_elt);
            count++;
        }
    }

    static inline void xml_write(const lane *lr, const size_t count, xmlpp::Element *elt)
    {
        xmlpp::Element *lr_elt = elt->add_child("lane_ref");
        lr_elt->set_attribute("ref", lr->id);
        lr_elt->set_attribute("local_id", boost::lexical_cast<str>(count));
    }

    static inline void xml_write(const intersection::state &is, const size_t count, xmlpp::Element *elt)
    {
        xmlpp::Element *is_elt = elt->add_child("state");
        is_elt->set_attribute("id", boost::lexical_cast<str>(count));
        is_elt->set_attribute("duration", boost::lexical_cast<str>(is.duration));

        BOOST_FOREACH(const intersection::state::state_pair &sp, is.in_pair())
        {
            xmlpp::Element *ii_elt = is_elt->add_child("lane_pair");
            ii_elt->set_attribute("in_id",  boost::lexical_cast<str>(sp.in_idx));
            ii_elt->set_attribute("out_id", boost::lexical_cast<str>(sp.out_idx));
        }
    }

    void intersection::xml_write(xmlpp::Element *elt) const
    {
        xmlpp::Element *intersection_elt = elt->add_child("intersection");
        xmlpp::Element *incident_elt     = intersection_elt->add_child("incident");

        intersection_elt->set_attribute("id", id);
        xml_write_vector(incoming, incident_elt, "incoming");
        xml_write_vector(outgoing, incident_elt, "outgoing");
        xml_write_vector(states, intersection_elt, "states");
    }

    void network::xml_write(const char *filename) const
    {
        xmlpp::Document out;

        xmlpp::Element *root(out.create_root_node("network"));
        xml_write(root);

        out.write_to_file_formatted(filename, "utf-8");
    }

    void network::xml_write(xmlpp::Element *elt) const
    {
        elt->set_attribute("name",    name);
        elt->set_attribute("version", "1.3");
        elt->set_attribute("gamma", boost::lexical_cast<str>(gamma));
        elt->set_attribute("lane_width", boost::lexical_cast<str>(lane_width));
        xml_write_map(roads,         elt, "roads");
        xml_write_map(lanes,         elt, "lanes");
        xml_write_map(intersections, elt, "intersections");
    }

    void network::svg_write(const char *filename, const int flags) const
    {
        xmlpp::Document out;
        out.set_internal_subset("svg", "-//W3C//DTD SVG 1.1//EN", "http://www.w3.org/Graphics/SVG/1.1/DTD/svg11.dtd");
        xmlpp::Element *root(out.create_root_node("svg", "http://www.w3.org/2000/svg"));

        vec3f l(FLT_MAX);
        vec3f h(-FLT_MAX);
        bounding_box(l, h);
        const float wi = h[0] - l[0];
        const float hi = h[1] - l[1];
        float width;
        float height;
        if(wi > hi)
        {
            width = 1;
            height = hi/wi;
        }
        else
        {
            height = 1;
            width = wi/hi;
        }
        float image_scale = 500;

        root->set_attribute("width", boost::str(boost::format("%fpx") % (image_scale*width)));
        root->set_attribute("height", boost::str(boost::format("%fpx") % (image_scale*height)));
        root->set_attribute("version", "1.1");

        xmlpp::Element *title = root->add_child("title");
        title->add_child_text(boost::str(boost::format("%s network") % name));
        xmlpp::Element *desc  = root->add_child("desc");
        desc->add_child_text(boost::str(boost::format("%s network") % name));

        xmlpp::Element *flipgroup = root->add_child("g");
        flipgroup->set_attribute("transform", boost::str(boost::format("scale(%11.8f,%11.8f) translate(%f,%f) scale(1, -1)") % (image_scale*width/wi) % (image_scale*height/hi) % h[0] % h[1]));

        if(flags & SVG_ROADS)
        {
            xmlpp::Element *roadgroup = flipgroup->add_child("g");
            roadgroup->set_attribute("id", "roads");

            xmlpp::Element *arcgroup = roadgroup->add_child("g");
            arcgroup->set_attribute("id", "arc_roads");
            arcgroup->set_attribute("fill", "none");
            arcgroup->set_attribute("stroke", "black");
            arcgroup->set_attribute("stroke-width", "0.5");

            xmlpp::Element *polygroup = roadgroup->add_child("g");
            polygroup->set_attribute("id", "poly_roads");
            polygroup->set_attribute("fill", "none");
            polygroup->set_attribute("stroke", "black");
            polygroup->set_attribute("stroke-width", "0.5");

            xmlpp::Element *arc_arcgroup, *arc_circlegroup;
            if(flags & SVG_ARCS)
            {
                arc_arcgroup = arcgroup->add_child("g");
                arc_arcgroup->set_attribute("id", "arcs");
                arc_arcgroup->set_attribute("fill", "none");
                arc_arcgroup->set_attribute("opacity", "1.0");
                arc_arcgroup->set_attribute("stroke", "blue");
            }
            if(flags & SVG_CIRCLES)
            {
                arc_circlegroup = arcgroup->add_child("g");
                arc_circlegroup->set_attribute("id", "circles");
                arc_circlegroup->set_attribute("fill", "none");
                arc_circlegroup->set_attribute("opacity", "1.0");
                arc_circlegroup->set_attribute("stroke", "red");
            }

            BOOST_FOREACH(const road_pair &rp, roads)
            {
                {
                    xmlpp::Element *path = arcgroup->add_child("path");
                    path->set_attribute("d", rp.second.rep.svg_arc_path(vec2f(0.0, 1.0), 0.0).stringify());
                    path->set_attribute("id", boost::str(boost::format("id%s_arc") % rp.first));
                }
                if(flags & SVG_ARCS)
                    rp.second.rep.svg_arc_arcs(rp.first, arc_arcgroup);
                if(flags & SVG_CIRCLES)
                    rp.second.rep.svg_arc_circles(rp.first, arc_circlegroup);
                {
                    xmlpp::Element *path = polygroup->add_child("path");
                    path->set_attribute("d", rp.second.rep.svg_poly_path(vec2f(0.0, 1.0), 0.0).stringify());
                    path->set_attribute("id", boost::str(boost::format("id%s_poly") % rp.first));
                }
            }

            std::tr1::unordered_map<const str, bool, hash<const str> > fict_road_map;
            BOOST_FOREACH(const intersection_pair &ip, intersections)
            {
                BOOST_FOREACH(const intersection::state &s, ip.second.states)
                {
                    intersection::state::state_pair_in::iterator current = s.in_pair().begin();
                    for(; current != s.in_pair().end(); ++current)
                    {
                        const intersection::state::state_pair &sp = *current;

                        const lane *in_lane  = ip.second.incoming[sp.in_idx];
                        const lane *out_lane = ip.second.outgoing[sp.out_idx];

                        const str id(boost::str(boost::format("id%s_to_%s") % in_lane->id % out_lane->id));

                        std::tr1::unordered_map<const str, bool, hash<const str> >::const_iterator f = fict_road_map.find(id);
                        if(f != fict_road_map.end())
                            continue;

                        fict_road_map.insert(std::make_pair(id, true));

                        const arc_road &in_road     = in_lane->road_memberships[1.0]->second.parent_road->rep;
                        const float     in_param    = in_lane->road_memberships[1.0]->second.interval[1];
                        const bool      in_reverse  = in_lane->road_memberships[1.0]->second.interval[0] > in_lane->road_memberships[1.0]->second.interval[1];

                        const arc_road &out_road    = out_lane->road_memberships[0.0]->second.parent_road->rep;
                        const float     out_param   = out_lane->road_memberships[0.0]->second.interval[0];
                        const bool      out_reverse = out_lane->road_memberships[0.0]->second.interval[0] > out_lane->road_memberships[0.0]->second.interval[1];

                        vec3f start_point;
                        vec3f start_tan;
                        vec3f end_point;
                        vec3f end_tan;
                        {
                            const mat4x4f start(in_road. point_frame(in_param,  0,  in_reverse));
                            const mat4x4f end  (out_road.point_frame(out_param, 0, out_reverse));
                            for(int i = 0; i < 3; ++i)
                            {
                                start_point[i] = start(i, 3);
                                start_tan[i]   = start(i, 0);
                                end_point[i]   = end(i, 3);
                                end_tan[i]     = -end(i, 0);
                            }
                        }
                        arc_road ar;
                        ar.initialize_from_polyline(0.0f, from_tan_pairs(start_point,
                                                                         start_tan,
                                                                         end_point,
                                                                         end_tan));
                        {
                            xmlpp::Element *path = arcgroup->add_child("path");
                            path->set_attribute("d", ar.svg_arc_path(vec2f(0.0, 1.0), 0.0).stringify()+"Z");
                            path->set_attribute("id", boost::str(boost::format("%s_arc") % id));
                        }
                        if(flags & SVG_ARCS)
                            ar.svg_arc_arcs(id, arc_arcgroup);
                        if(flags & SVG_CIRCLES)
                            ar.svg_arc_circles(id, arc_circlegroup);
                        {
                            xmlpp::Element *path = polygroup->add_child("path");
                            path->set_attribute("d", ar.svg_poly_path(vec2f(0.0, 1.0), 0.0).stringify()+"Z");
                            path->set_attribute("id", boost::str(boost::format("%s_poly") % id));
                        }
                    }
                }
            }
        }

        if(flags & SVG_LANES)
        {
            xmlpp::Element *lanegroup = flipgroup->add_child("g");
            lanegroup->set_attribute("id", "lanes");

            xmlpp::Element *arcgroup = lanegroup->add_child("g");
            arcgroup->set_attribute("id", "arc_lanes");
            arcgroup->set_attribute("fill", "none");
            arcgroup->set_attribute("stroke", "black");
            arcgroup->set_attribute("stroke-width", "0.5");

            xmlpp::Element *polygroup = lanegroup->add_child("g");
            polygroup->set_attribute("id", "poly_lanes");
            polygroup->set_attribute("fill", "none");
            polygroup->set_attribute("stroke", "black");
            polygroup->set_attribute("stroke-width", "0.5");

            BOOST_FOREACH(const lane_pair &lp, lanes)
            {
                {
                    xmlpp::Element *path = arcgroup->add_child("path");
                    path->set_attribute("d", lp.second.svg_arc_path(lane_width).stringify()+"Z");
                    path->set_attribute("id", boost::str(boost::format("id%s_arc") % lp.first));
                }
                {
                    xmlpp::Element *path = polygroup->add_child("path");
                    path->set_attribute("d", lp.second.svg_poly_path(lane_width).stringify()+"Z");
                    path->set_attribute("id", boost::str(boost::format("id%s_poly") % lp.first));
                }
            }

            BOOST_FOREACH(const intersection_pair &ip, intersections)
            {
                BOOST_FOREACH(const intersection::state &s, ip.second.states)
                {
                    BOOST_FOREACH(const lane_pair &lp, s.fict_lanes)
                    {
                        {
                            xmlpp::Element *path = arcgroup->add_child("path");
                            path->set_attribute("d", lp.second.svg_arc_path(lane_width).stringify()+"Z");
                            path->set_attribute("id", boost::str(boost::format("id%s_arc") % lp.first));
                        }
                        {
                            xmlpp::Element *path = polygroup->add_child("path");
                            path->set_attribute("d", lp.second.svg_poly_path(lane_width).stringify()+"Z");
                            path->set_attribute("id", boost::str(boost::format("id%s_poly") % lp.first));
                        }
                    }
                }
            }
        }

        out.write_to_file_formatted(filename, "utf-8");
    }
}
