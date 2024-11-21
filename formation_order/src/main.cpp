#include "formation_order.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "formation_order_node");
    ros::NodeHandle nh;

    formation_order fo(nh);

    fo.param_read();
    fo.init();
    fo.run();

    return 0;
}