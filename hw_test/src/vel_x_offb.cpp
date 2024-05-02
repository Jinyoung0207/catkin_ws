//X 축으로만 0.2의 속도로 이동 (시뮬레이션 용)
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Path.h>
#include <string>
#include <iostream>
#include <cmath>
#include <queue>
#include <numeric>


class X_velocity
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber pose_sub;
    ros::Subscriber state_sub; 
    
    ros::ServiceClient arming_client;
    ros::ServiceClient set_mode_client;
    
    ros::Publisher cmd_vel_pub;
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
    
    
public:
    X_velocity() : nh_(""),current_state(),current_pose()
    {
        pose_sub = nh_.subscribe<geometry_msgs::PoseStamped>("UAV0/mavros/local_position/pose", 10, &X_velocity::poseCallback, this);
        state_sub = nh_.subscribe<mavros_msgs::State>("UAV0/mavros/state", 10, &X_velocity::stateCallback, this);
        cmd_vel_pub = nh_.advertise<geometry_msgs::TwistStamped>("/UAV0/mavros/setpoint_velocity/cmd_vel", 10);
        arming_client = nh_.serviceClient<mavros_msgs::CommandBool>("UAV0/mavros/cmd/arming");
        set_mode_client= nh_.serviceClient<mavros_msgs::SetMode>("UAV0/mavros/set_mode");
    }

    void run()
    {
        ros::Rate rate(10);
        double x,y,z = 10;

        geometry_msgs::TwistStamped vel;
        vel.twist.linear.x = 0;
        vel.twist.linear.y = 0;
        vel.twist.linear.z = 0.2;

       
        int count = 0;
        while (ros::ok()) 
        {
            geometry_msgs::PoseStamped add_pose;
            if(count == 0)
            {    
                for(int a=0;a<100;a++)
                {
                    add_pose.pose.position.x += current_pose.pose.position.x;
                    add_pose.pose.position.y += current_pose.pose.position.y;
                    add_pose.pose.position.z += current_pose.pose.position.z;
                    count++;
                }
                double aver_x = add_pose.pose.position.x / 100;
                double aver_y = add_pose.pose.position.y / 100;
                double aver_z = add_pose.pose.position.z / 100;
                ROS_INFO("Averaging is done : aver_x %f aver_y %f aver_z %f",aver_x,aver_y,aver_z);
            }
            
            if(current_pose.pose.position.z <= (add_pose.pose.position.z/100)+ 1.8)
            {
                vel.twist.linear.x = 0;
                vel.twist.linear.y = 0;
                vel.twist.linear.z = 0.5;
            }
            else if(current_pose.pose.position.z >= add_pose.pose.position.z/100 + 1.8)
            {
                vel.twist.linear.x = 0.2;
                vel.twist.linear.y = 0;
                vel.twist.linear.z = 2-current_pose.pose.position.z;
            }
            cmd_vel_pub.publish(vel);
            ros::spinOnce();
            rate.sleep();
        }
    }
};
           
int main(int argc, char** argv)
{
    ros::init(argc, argv, "x_vel_offb");
    X_velocity x_vel;
    x_vel.run();

    return 0;
}    
    
    
    
    
    
    
    
    

