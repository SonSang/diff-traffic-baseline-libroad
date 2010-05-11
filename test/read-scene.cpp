#include "libroad/hwm_network.hpp"

int main(int argc, char *argv[])
{
    std::cout << libroad_package_string() << std::endl;
    if(argc < 2)
    {
        std::cerr << "Usage: " << argv[0] << " <input scene> " << std::endl;
        return 1;
    }

    xml_read_scene(argv[1]);

    return 0;
}
