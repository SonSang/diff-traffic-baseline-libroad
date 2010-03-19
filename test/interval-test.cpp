#include "libroad/partition01.hpp"

#include <iostream>
#include <fstream>
#include <string>
#include <boost/foreach.hpp>

int main(int argc, char *argv[])
{
    std::cerr << libroad_package_string() << std::endl;

    partition01<std::string> test_int;

    test_int.insert(0.0f, "zero point zero");
    test_int.insert(0.4f, "zero point four");
    test_int.insert(0.7f, "zero point seven");

    std::cout << test_int.find(0.1)->second << std::endl;
    std::cout << test_int.find(0.5)->second << std::endl;
    std::cout << test_int.find(0.7)->second << std::endl;
    std::cout << test_int.find(0.8)->second << std::endl;

    BOOST_FOREACH(partition01<std::string>::entry &et, test_int)
    {
        et.second += "duck";
    }

    BOOST_FOREACH(const partition01<std::string>::entry &et, test_int)
    {
        std::cout << et.first << ": " << et.second << std::endl;
    }

    for(partition01<std::string>::iterator v = test_int.begin();
        v != test_int.end();
        ++v)
    {
        std::cout << test_int.containing_interval(v) << std::endl;
    }

    for(partition01<std::string>::reverse_iterator v = test_int.rbegin();
        v != test_int.rend();
        ++v)
    {
        std::cout << test_int.containing_interval(v) << std::endl;
    }

    std::cout << test_int.interval_length(test_int.find(0.8)) << std::endl;

    return 0;
}
