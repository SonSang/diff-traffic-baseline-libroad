#include <libroad/osm_network.hpp>
#include <libroad/hwm_network.hpp>

int main(int argc, char *argv[])
{
    std::cerr << libroad_package_string() << std::endl;
    if(argc < 5)
    {
        std::cerr << "Usage: " << argv[0] << " <x nodes> <y nodes> <scale> <output file>" << std::endl;
        return 1;
    }

    const int x_nodes = boost::lexical_cast<int>(argv[1]);
    if(x_nodes < 2)
    {
        std::cerr << "x_nodes must be greater than 1" << std::endl;
        return 1;
    }

    const int y_nodes = boost::lexical_cast<int>(argv[2]);
    if(y_nodes < 2)
    {
        std::cerr << "y_nodes must be greater than 1" << std::endl;
        return 1;
    }

    const float scale = boost::lexical_cast<float>(argv[3]);
    if(scale < 5)
    {
        std::cerr << "Scale of grid must be greater than 5" << std::endl;
        return 1;
    }

    osm::network onet;
    onet.create_grid(x_nodes, y_nodes, scale*x_nodes, scale*y_nodes);
    onet.compute_edge_types();
    onet.compute_node_degrees();
    onet.join_logical_roads();
    onet.split_into_road_segments();
    onet.remove_small_roads(15);
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

    net.xml_write(argv[4]);

    return 0;
}
