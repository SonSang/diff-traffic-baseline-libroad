#include "hwm-net-mesh.hpp"

int main(int argc, char *argv[])
{
    std::cerr << libroad_package_string() << std::endl;

    if(argc < 3)
    {
        std::cerr << "Usage: " << argv[0] << " <input network> <output obj>" << std::endl;
        return 1;
    }
    hwm::network net(hwm::load_xml_network(argv[1], vec3f(1.0, 1.0, 1.0f)));

    net.build_intersections();
    net.build_fictitious_lanes();
    net.auto_scale_memberships();
    net.center();
    std::cerr << "HWM net loaded successfully" << std::endl;

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

    hwm::network_to_obj(argv[2], net);

    return 0;
}
