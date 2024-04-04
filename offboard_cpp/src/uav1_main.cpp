#include "uav1_move.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "uav1_move_node");

    Offboard offboard1;
    offboard1.run();

    return 0;
}