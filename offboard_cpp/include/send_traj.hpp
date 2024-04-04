#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <cmath>

class SendTrajectory
{
public:
    SendTrajectory() : nh_(""), current_pose_()
    {
        pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("uav0/mavros/local_position/pose", 10, &SendTrajectory::poseCallback, this);

        local_pos_pub_0 = nh_.advertise<geometry_msgs::PoseStamped>("uav0/mavros/setpoint_position/local", 10);
        local_pos_pub_1 = nh_.advertise<geometry_msgs::PoseStamped>("uav1/mavros/setpoint_position/local", 10);
        local_pos_pub_2 = nh_.advertise<geometry_msgs::PoseStamped>("uav2/mavros/setpoint_position/local", 10);
    }
    void run();

private:
    ros::NodeHandle nh_;
    ros::Subscriber pose_sub_;
    ros::Publisher local_pos_pub_0;
    ros::Publisher local_pos_pub_1;
    ros::Publisher local_pos_pub_2;

    geometry_msgs::PoseStamped current_pose_;
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) 
    {
        current_pose_ = *msg;
    }
};
    


