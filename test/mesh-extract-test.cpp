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

    bf::path full_out_path(argv[2]);

    bf::path dir(full_out_path.parent_path());
    bf::path tex_dir(full_out_path.parent_path() / "tex");

    if(!dir.empty())
       bf::create_directory(dir);
    bf::remove_all(tex_dir);
    bf::create_directory(tex_dir);

    if(!dir.empty())
        bf::current_path(dir);
    std::ofstream out(full_out_path.filename().c_str());
    obj_roads(out, net);

    BOOST_FOREACH(const hwm::intersection_pair &ip, net.intersections)
    {
        obj_intersection(out, ip.second);
    }

    return 0;
}
