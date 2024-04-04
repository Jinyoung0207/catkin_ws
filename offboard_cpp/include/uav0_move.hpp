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
    Offboard() : nh_(""), current_pose_(), current_state_(),setpoint_pose_()
    {
        state_sub_ = nh_.subscribe<mavros_msgs::State>("uav0/mavros/state", 10, &Offboard::stateCallback, this);
        local_pos_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("uav0/mavros/setpoint_position/local", 10, &Offboard::setposeCallback, this);//드론의 setpoint 구독 
        pose_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>("uav0/mavros/local_position/pose", 10, &Offboard::poseCallback, this);

        path_pub_ = nh_.advertise<nav_msgs::Path>("uav0/path", 10); //path
        point_pub_ = nh_.advertise<geometry_msgs::PointStamped>("uav0/point", 10); //p
        
        arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("uav0/mavros/cmd/arming");
        set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("uav0/mavros/set_mode");
    }
    void run();
private:
    ros::NodeHandle nh_;
    ros::Subscriber state_sub_;
    ros::Subscriber local_pos_sub_; 
    ros::Subscriber pose_sub_;
    ros::Publisher path_pub_; //path
    ros::Publisher point_pub_; //p
    ros::ServiceClient arming_client_;
    ros::ServiceClient set_mode_client_;
    mavros_msgs::State current_state_;
    geometry_msgs::PoseStamped setpoint_pose_;
    geometry_msgs::PoseStamped current_pose_;
    geometry_msgs::PointStamped point; //p
    nav_msgs::Path path; //path
    void stateCallback(const mavros_msgs::State::ConstPtr& msg);
        

    void setposeCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
        

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg); 

    
};