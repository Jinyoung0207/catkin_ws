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



class Velocity
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber pose_sub;
    ros::Subscriber state_sub; 
    ros::Subscriber ref_path_sub;

    ros::ServiceClient arming_client;
    ros::ServiceClient set_mode_client;
    
    ros::Publisher cmd_vel_pub;
    mavros_msgs::State current_state;
    nav_msgs::Path ref_path_msg;
    geometry_msgs::PoseStamped current_pose;
    

    void stateCallback(const mavros_msgs::State::ConstPtr& msg) 
    {
        current_state = *msg;
    }

    void refpathCallback(const nav_msgs::Path::ConstPtr& msg) 
    {
        ref_path_msg = *msg;
    }

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) 
    {
        current_pose = *msg;
    }
public:
    Velocity() : nh_(""),current_state(),current_pose(),ref_path_msg()
    {   //각각 0,1,2의 위치 구독함으로써 거리 계산 가능
        pose_sub = nh_.subscribe<geometry_msgs::PoseStamped>("uav2/mavros/local_position/pose", 10, &Velocity::poseCallback, this);
        ref_path_sub = nh_.subscribe<nav_msgs::Path>("uav2/ref_path", 10, &Velocity::refpathCallback, this);
        state_sub = nh_.subscribe<mavros_msgs::State>("uav2/mavros/state", 10, &Velocity::stateCallback, this);
        cmd_vel_pub = nh_.advertise<geometry_msgs::TwistStamped>("/uav2/mavros/setpoint_velocity/cmd_vel", 10);
        arming_client = nh_.serviceClient<mavros_msgs::CommandBool>("uav2/mavros/cmd/arming");
        set_mode_client= nh_.serviceClient<mavros_msgs::SetMode>("uav2/mavros/set_mode");
        // Initialize PID controllers with P and I gains
    }
    
    void run()
    {
        ros::Rate rate(10);

        double x,y,z = 0; //ey = 현재 오차 거리 vel_x는 pi계산되어서 나온 x축 선속도
     
        geometry_msgs::TwistStamped vel;
        vel.twist.linear.x=0;
        vel.twist.linear.y=0;
        vel.twist.linear.z=0.2;

        // Wait for FCU connection
        while (ros::ok() && !current_state.connected) 
        {
            cmd_vel_pub.publish(vel);
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
                    ROS_INFO("uav2 Offboard enabled");
                }
                last_request = ros::Time::now();
            } 
            else
            {
                if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))) 
                {
                    if (arming_client.call(arm_cmd) && arm_cmd.response.success) 
                    {
                        ROS_INFO("uav2 armed");
                    }
                    last_request = ros::Time::now();
                }
            }
            x = ref_path_msg.poses[0].pose.position.x-current_pose.pose.position.x+3;
            y = ref_path_msg.poses[0].pose.position.y-current_pose.pose.position.y+3;
            z = ref_path_msg.poses[0].pose.position.z-current_pose.pose.position.z;
            
           

          
            vel.twist.linear.x = x;
            vel.twist.linear.y = y;
            vel.twist.linear.z = z;
            cmd_vel_pub.publish(vel);      
            
            ros::spinOnce();
            rate.sleep();         
        }  
    }
};     



int main(int argc, char** argv)
{
    ros::init(argc, argv, "velocity_pub2");
    Velocity velocity_pub;
    velocity_pub.run();

    return 0;

}
