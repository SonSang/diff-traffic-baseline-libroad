#include <Magick++.h>
#include <boost/format.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/regex.hpp>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <iostream>

namespace bf = boost::filesystem;

int main(int argc, char *argv[])
{
    if(argc != 3)
    {
        std::cerr << "Usage: " << argv[0] << " <input dir> <output file>" << std::endl;
        return 1;
    }

    const boost::regex re(".*\\.png");
    const boost::regex num_regex("Sat_x=(\\d*)y=(\\d*)zoom=\\d*");

    size_t lowest[2] = {std::numeric_limits<size_t>::max(),
                        std::numeric_limits<size_t>::max()};
    size_t highest[2] = {0,
                         0};

    bool have_size = false;
    int tile_x;
    int tile_y;

    bf::directory_iterator end_itr;
    for( bf::directory_iterator itr(argv[1]);
         itr != end_itr;
         ++itr)
    {
        if(itr->path().has_filename() && boost::regex_match(itr->path().filename(), re))
        {
            const std::string &name(itr->path().stem());

            boost::smatch what;
            boost::regex_match(name, what, num_regex);
            if(what.size() != 3)
                continue;

            const size_t x = boost::lexical_cast<size_t>(what[1]);
            const size_t y = boost::lexical_cast<size_t>(what[2]);

            Magick::Image im(itr->path().string());
            int this_x = im.columns();
            int this_y = im.rows();
            if(!have_size)
            {
                have_size = true;
                tile_x = this_x;
                tile_y = this_y;
            }
            else
            {
                assert(tile_x == this_x);
                assert(tile_y == this_y);
            }

            lowest[0]      = std::min(lowest[0], x);
            lowest[1]      = std::min(lowest[1], y);
            highest[0]     = std::max(highest[0], x);
            highest[1]     = std::max(highest[1], y);
        }
    }

    std::cout << "lowest  tile # " << lowest[0] << " " << lowest[1] << std::endl;
    std::cout << "highest tile # " << highest[0] << " " << highest[1] << std::endl;

    const size_t xtiles = highest[0] - lowest[0];
    const size_t ytiles = highest[1] - lowest[1];

    std::cout << "each tile is # " << tile_x << " x " << tile_y << std::endl;
    std::cout << "resulting image will be " << xtiles*tile_x << " x " << ytiles*tile_y << std::endl;

    const Magick::Geometry geo(xtiles*tile_x, ytiles*tile_y);
    Magick::Image result(geo, Magick::ColorRGB(0.0, 0.0, 0.0));

    std::cout << "built empty image, composing..." << std::endl;

    for(bf::directory_iterator itr(argv[1]);
         itr != end_itr;
         ++itr)
    {
        if(itr->path().has_filename() && boost::regex_match(itr->path().filename(), re))
        {
            const std::string &name(itr->path().stem());

            boost::smatch what;
            boost::regex_match(name, what, num_regex);
            if(what.size() != 3)
                continue;

            const size_t x = boost::lexical_cast<size_t>(what[1]);
            const size_t y = boost::lexical_cast<size_t>(what[2]);

            std::cout << "composing " << x << "," << y << " -> " << x - lowest[0] << ", " << y - lowest[1] << std::endl;

            Magick::Image im(itr->path().string());
            int this_x = im.columns();
            int this_y = im.rows();
            assert(have_size);
            assert(tile_x == this_x);
            assert(tile_y == this_y);

            result.composite(im, (x-lowest[0])*tile_x, (y-lowest[1])*tile_y, Magick::OverCompositeOp);
        }
    }

    std::cout << "done composing; writing " << argv[2] << std::endl;

    result.write(argv[2]);

    std::cout << "done." << std::endl;

    return 0;
}
