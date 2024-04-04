#include "uav2_move.hpp"

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "uav2_move_node");

    Offboard offboard2;
    offboard2.run();

    return 0;
}