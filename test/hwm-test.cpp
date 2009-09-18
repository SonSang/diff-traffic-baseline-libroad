#include <libroad/hwm_network.hpp>
#include <iostream>

#include <boost/foreach.hpp>

int main(int argc, char *argv[])
{
    hwm::network net;

    if(!net.xml_read(argv[1]))
    {
        std::cerr << "HWM net failed to load: " << argv[1] << std::endl;
        return 1;
    }
    else
        std::cerr << "HWM net loaded successfully" << std::endl;

    return 0;
}
