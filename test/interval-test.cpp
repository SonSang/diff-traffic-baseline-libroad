#include "libroad/partition01.hpp"

#include <iostream>
#include <fstream>
#include <string>
#include <boost/foreach.hpp>

int main(int argc, char *argv[])
{
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

    std::cout << test_int.interval_length(test_int.find(0.8)) << std::endl;

    return 0;
}
