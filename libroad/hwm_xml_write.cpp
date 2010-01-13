#include "hwm_network.hpp"
#include "xml_util.hpp"

namespace hwm
{
    template <class T>
    static inline void xml_write_map(const T &v, xmlpp::Element *elt, const str &name)
    {
        xmlpp::Element *map_elt = elt->add_child(name);

        typedef typename T::value_type val;
        BOOST_FOREACH(const val &item, v)
        {
            xml_write(item.second, map_elt);
        }
    }

    static inline void xml_write(const polyline_road &pr, xmlpp::Element *elt)
    {
        xmlpp::Element *pr_elt = elt->add_child("line_rep");
        xmlpp::Element *pt_elt = pr_elt->add_child("points");

        BOOST_FOREACH(const vec3f &pt, pr.points_)
        {
            pt_elt->add_child_text(boost::str(boost::format("%f %f %f 0.0\n") % pt[0] % pt[1] % pt[2]));
        }
    }

    static inline void xml_write(const road &r, xmlpp::Element *elt)
    {
        xmlpp::Element *road_elt = elt->add_child("road");
        road_elt->set_attribute("id", r.id);
        road_elt->set_attribute("name", r.name);
        xml_write(r.rep, road_elt);
    }

    static inline void xml_write(const lane::terminus &lt, xmlpp::Element *elt, const str &name)
    {
        xmlpp::Element *term_elt = elt->add_child(name);
        if(!lt.inters)
            term_elt->add_child("dead_end");
        else
        {
            xmlpp::Element *iref = term_elt->add_child("intersection_ref");
            iref->set_attribute("ref", lt.inters->id);
        }
    }

    static inline void xml_write_empty(const lane::adjacency &rm, xmlpp::Element *elt)
    {
        xmlpp::Element *rm_elt = elt->add_child("lane_adjacency");
    }

    static inline void xml_write_empty(const lane::road_membership &rm, xmlpp::Element *elt)
    {
        xmlpp::Element *rm_elt = elt->add_child("road_membership");
    }

    template <class T>
    static inline void xml_write_partition(const partition01<T> &p, xmlpp::Element *elt, const str &name)
    {
        xmlpp::Element *overall_elt = elt->add_child(name);
        xmlpp::Element *interval_elt = overall_elt->add_child("interval");

        typename partition01<T>::const_iterator pit = p.begin();
        xmlpp::Element *base_elt = interval_elt->add_child("base");
        if(!p.empty())
        {
            xml_write(pit->second, base_elt);
            for(++pit; pit != p.end(); ++pit)
            {
                xmlpp::Element *div_elt = interval_elt->add_child("divider");
                div_elt->set_attribute("value", boost::lexical_cast<str>(pit->first));
                xml_write(pit->second, div_elt);
            }
        }
        else
        {
            xml_write_empty(pit->second, base_elt);
        }
    }


    static inline void xml_write(const lane::road_membership &rm, xmlpp::Element *elt)
    {
        xmlpp::Element *rm_elt = elt->add_child("road_membership");
        if(!rm.empty())
        {
            rm_elt->set_attribute("parent_road_ref", rm.parent_road->id);
            rm_elt->set_attribute("interval_start", boost::lexical_cast<str>(rm.interval[0]));
            rm_elt->set_attribute("interval_end", boost::lexical_cast<str>(rm.interval[1]));
            rm_elt->set_attribute("lane_position", boost::lexical_cast<str>(rm.lane_position));
        }
    }

    static inline void xml_write(const lane::adjacency &la, xmlpp::Element *elt)
    {
        xmlpp::Element *la_elt = elt->add_child("lane_adjacency");
        if(!la.empty())
        {
            la_elt->set_attribute("lane_ref", la.neighbor->id);
            la_elt->set_attribute("interval_start", boost::lexical_cast<str>(la.neighbor_interval[0]));
            la_elt->set_attribute("interval_end", boost::lexical_cast<str>(la.neighbor_interval[1]));
        }
    }

    static inline void xml_write(const lane &l, xmlpp::Element *elt)
    {
        xmlpp::Element *lane_elt = elt->add_child("lane");
        lane_elt->set_attribute("id", l.id);
        lane_elt->set_attribute("speedlimit", boost::lexical_cast<str>(l.speedlimit));
        xml_write(l.start, lane_elt, "start");
        xml_write(l.end, lane_elt, "end");
        xml_write_partition(l.road_memberships, lane_elt, "road_intervals");
        xmlpp::Element *adj_elt = lane_elt->add_child("adjacency_intervals");
        xml_write_partition(l.left,  adj_elt, "left");
        xml_write_partition(l.right, adj_elt, "right");
    }

    template <class T>
    static inline void xml_write(const std::vector<T> &v, xmlpp::Element *elt, const str &name)
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

        size_t oref = 0;
        std::cout << is.out_states.size() << " now size" << std::endl;

        BOOST_FOREACH(const intersection::state::in_id &ii, is.out_states)
        {
            if(ii.in_ref != -1)
            {
                xmlpp::Element *ii_elt = is_elt->add_child("lane_pair");
                ii_elt->set_attribute("in_id", boost::lexical_cast<str>(ii.in_ref));
                ii_elt->set_attribute("out_id", boost::lexical_cast<str>(oref));
            }
            ++oref;
        }
    }

    static inline void xml_write(const intersection &i, xmlpp::Element *elt)
    {
        xmlpp::Element *intersection_elt = elt->add_child("intersection");
        xmlpp::Element *incident_elt     = intersection_elt->add_child("incident");

        intersection_elt->set_attribute("id", i.id);
        xml_write(i.incoming, incident_elt, "incoming");
        xml_write(i.outgoing, incident_elt, "outgoing");
        xml_write(i.states, intersection_elt, "states");
    }

    void write_xml_network(const network &n, const char *filename)
    {
        xmlpp::Document out;

        xmlpp::Element *root(out.create_root_node("network"));
        root->set_attribute("name",    n.name);
        root->set_attribute("version", "1.2");
        root->set_attribute("gamma", boost::lexical_cast<str>(n.gamma));
        xml_write_map(n.roads, root, "roads");
        xml_write_map(n.lanes, root, "lanes");
        xml_write_map(n.intersections, root, "intersections");

        out.write_to_file_formatted(filename);
    }
}

