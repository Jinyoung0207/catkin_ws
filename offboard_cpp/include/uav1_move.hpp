#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Path.h> //path
#include <geometry_msgs/PointStamped.h> //p

class Offboard
{
public:
    Offboard() : nh_(""), current_pose_(),current_pose_1(), current_state_(),setpoint_pose_() 
    {
        state_sub_ = nh_.subscribe<mavros_msgs::State>("uav1/mavros/state", 10, &Offboard::stateCallback, this);
        local_pos_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("uav1/mavros/setpoint_position/local", 10, &Offboard::setposeCallback, this);
        pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("uav0/mavros/local_position/pose", 10, &Offboard::poseCallback, this);
        pose_sub_1 = nh_.subscribe<geometry_msgs::PoseStamped>("uav1/mavros/local_position/pose", 10, &Offboard::pose1Callback, this);

        path_pub_ = nh_.advertise<nav_msgs::Path>("uav1/path", 10); //path
        point_pub_ = nh_.advertise<geometry_msgs::PointStamped>("uav1/point", 10); //p
        pcp=nh_.advertise<geometry_msgs::PoseStamped>("uav1/pose1", 10);

        arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("uav1/mavros/cmd/arming");
        set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("uav1/mavros/set_mode");
    }
    void run();

    double x1,y1 = 0;

private:
    ros::NodeHandle nh_;
    ros::Subscriber state_sub_;
    ros::Subscriber local_pos_sub_; 
    ros::Subscriber pose_sub_;
    ros::Subscriber pose_sub_1;
    ros::Publisher path_pub_; //path
    ros::Publisher point_pub_; //p
    ros::Publisher pcp;
    ros::ServiceClient arming_client_;
    ros::ServiceClient set_mode_client_;
    mavros_msgs::State current_state_;
    geometry_msgs::PoseStamped setpoint_pose_;
    geometry_msgs::PoseStamped current_pose_;
    geometry_msgs::PoseStamped current_pose_1;
    geometry_msgs::PointStamped point; //p
    geometry_msgs::PoseStamped cp;
    nav_msgs::Path path; //path

    void stateCallback(const mavros_msgs::State::ConstPtr& msg);
    void setposeCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void pose1Callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
};