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
}

