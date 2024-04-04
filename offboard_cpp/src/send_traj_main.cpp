#include "send_traj.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "send_traj_node");

    SendTrajectory send_traj;
    send_traj.run();

    return 0;
}