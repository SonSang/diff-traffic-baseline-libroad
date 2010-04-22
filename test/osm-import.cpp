#include <libroad/osm_network.hpp>
#include <libroad/hwm_network.hpp>

int main(int argc, char *argv[])
{
    std::cerr << libroad_package_string() << std::endl;
    if(argc < 3)
    {
        std::cerr << "Usage: " << argv[0] << " <input osm> <output file>" << std::endl;
        return 1;
    }

    //Load from file
    osm::network onet(osm::load_xml_network(argv[1]));
    onet.populate_edges_from_hash();
    onet.remove_duplicate_nodes();
    onet.compute_node_degrees();
    onet.clip_roads_to_bounds();
    onet.compute_edge_types();
    onet.compute_node_degrees();
    onet.scale_and_translate();
    onet.split_into_road_segments();
    onet.remove_highway_intersections();
    onet.compute_node_heights();
    onet.join_logical_roads();
    onet.create_ramps();
    onet.remove_small_roads(50);
    onet.create_intersections();
    onet.populate_edge_hash_from_edges();

    hwm::network net(hwm::from_osm("test", 0.5f, 2.5, onet));
    net.build_intersections();
    net.build_fictitious_lanes();
    net.auto_scale_memberships();

    try
    {
        net.check();
        std::cerr << "HWM net checks out" << std::endl;
    }
    catch(std::runtime_error &e)
    {
        std::cerr << "HWM net doesn't check out: " << e.what() << std::endl;
        exit(1);
    }

    net.xml_write(argv[2]);

    return 0;
}
