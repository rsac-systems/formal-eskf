// SPDX-License-Identifier: BSD-3-Clause
#include "io.hpp"
#include "session.hpp"
#include <exception>
#include <iostream>
#include <string_view>

int main(int argc, char ** argv)
{
    try
    {
        if (argc == 2 && std::string_view(argv[1]) == "--geodesy")
        {
            replay_demo::project_coordinates(std::cin, std::cout);
            return 0;
        }
        if (argc == 4 && std::string_view(argv[3]) == "--kitti-session")
        {
            replay_demo::replay_session(argv[1], argv[2], false, true);
            return 0;
        }
        if ((argc == 4 || (argc == 5 && std::string_view(argv[4]) == "--no-mag")) &&
            std::string_view(argv[3]) == "--session")
        {
            replay_demo::replay_session(argv[1], argv[2], argc == 5);
            return 0;
        }
        std::cerr << "Usage: replay_ins INPUT_DIR NEW_OUTPUT_DIR --session [--no-mag]\n"
                     "       replay_ins INPUT_DIR NEW_OUTPUT_DIR --kitti-session\n"
                     "       replay_ins --geodesy < GEODETIC_ROWS\n";
        return 2;
    }
    catch (std::exception const & error)
    {
        std::cerr << "error: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
