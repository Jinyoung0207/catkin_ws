#include "uav0_move.hpp"

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "uav0_move_node");

    Offboard offboard;
    offboard.run();

    return 0;
}
