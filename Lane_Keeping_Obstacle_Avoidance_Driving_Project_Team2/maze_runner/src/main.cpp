#include "maze_runner/maze_drive.hpp"

using PREC = float;

int32_t main(int32_t argc, char** argv)
{
    ros::init(argc,argv,"Maze Runner");
    Maze::maze_drive<PREC> maze_runner;
    maze_runner.run();

    return 0;
}