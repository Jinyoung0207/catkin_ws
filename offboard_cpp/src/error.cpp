#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Path.h> //path
#include <string>
#include <iostream>
#include <cmath>
#include <queue>
#include <numeric>
#include <std_msgs/Int64.h>
#include <experimental/filesystem>

class Error 
{
private:
    ros::NodeHandle nh_;
    ros::Publisher error_pub;

    ros::Subscriber vel_sub;
    ros::Subscriber mav_vel_sub; //경로 생성 노드에서 보내는 경로를 구독해서 set_point_pub으로 발행하는거임
    geometry_msgs::TwistStamped setvel;
    geometry_msgs::TwistStamped mav_vel;

    ros::ServiceClient arming_client;
    ros::ServiceClient set_mode_client;

    ros::Subscriber pose_sub;
    ros::Subscriber state_sub; 
    mavros_msgs::State current_state;
    geometry_msgs::PoseStamped current_pose;
    
    void stateCallback(const mavros_msgs::State::ConstPtr& msg) 
    {
        current_state = *msg;
    }
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) 
    {
        current_pose = *msg;
    }

    void setvelCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
    {
        setvel = *msg;
    }
    void mavvelCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
    {
        mav_vel = *msg;
    }
     
public:
    Error() : nh_(""),setvel(),mav_vel()
    {
        vel_sub = nh_.subscribe<geometry_msgs::TwistStamped>("/uav0/mavros/setpoint_velocity/cmd_vel", 10, &Error::setvelCallback, this);
        mav_vel_sub = nh_.subscribe<geometry_msgs::TwistStamped>("uav0/mavros/local_position/velocity_local", 10, &Error::mavvelCallback, this);
        error_pub = nh_.advertise<geometry_msgs::TwistStamped>("error", 10);
        arming_client = nh_.serviceClient<mavros_msgs::CommandBool>("uav0/mavros/cmd/arming");
        set_mode_client= nh_.serviceClient<mavros_msgs::SetMode>("uav0/mavros/set_mode");
        pose_sub = nh_.subscribe<geometry_msgs::PoseStamped>("uav0/mavros/local_position/pose", 10, &Error::poseCallback, this);
        state_sub = nh_.subscribe<mavros_msgs::State>("uav0/mavros/state", 10, &Error::stateCallback, this);


    } 
    void run()
    {
        ros::Rate rate(10);

        double x,y,z = 0; //error 
        geometry_msgs::TwistStamped error;
        error.twist.linear.x=0;
        error.twist.linear.y=0;
        error.twist.linear.z=0.2;

        while (ros::ok() && !current_state.connected) 
        {
            error_pub.publish(error);
            ros::spinOnce();
            rate.sleep();
        }
        mavros_msgs::SetMode offb_set_mode;
        offb_set_mode.request.custom_mode = "OFFBOARD";

        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = true;

        ros::Time last_request = ros::Time::now();
        while (ros::ok()) 
        {
            if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))) 
            {
                if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) 
                {
                    ROS_INFO("Error pub is ready");
                }
                last_request = ros::Time::now();
            } 
            else
            {
                if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))) 
                {
                    if (arming_client.call(arm_cmd) && arm_cmd.response.success) 
                    {
                        ROS_INFO("really ready");
                    }
                    last_request = ros::Time::now();
                }
            }

            x = setvel.twist.linear.x - mav_vel.twist.linear.x;
            y = setvel.twist.linear.y - mav_vel.twist.linear.y;
            z = setvel.twist.linear.x - mav_vel.twist.linear.z;
        
            error.twist.linear.x = x;
            error.twist.linear.y = y;
            error.twist.linear.z = z;

            error_pub.publish(error);

            ros::spinOnce();
            rate.sleep();
        }    
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "error");
    Error error_pub;
    error_pub.run();

    return 0;

}