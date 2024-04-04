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


class Formation
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber pose0_sub;

    ros::Publisher num_pub;

    geometry_msgs::PoseStamped current_pose_0;
    std_msgs::Int64 formation_num;

    void pose0Callback(const geometry_msgs::PoseStamped::ConstPtr& msg) 
    {
        current_pose_0 = *msg;
    }
public:
    Formation() : nh_(""),current_pose_0()
    {
        pose0_sub = nh_.subscribe<geometry_msgs::PoseStamped>("uav0/mavros/local_position/pose", 10, &Formation::pose0Callback, this);
        num_pub = nh_.advertise<std_msgs::Int64>("num/formation/info", 10);
    }
    void run() 
    {
        ros::Rate rate(10.0);

        int num=1;
        int i=0;
        std_msgs::Int64 formation_num;
        while (ros::ok()) 
        {
            if (current_pose_0.pose.position.z >= 2.0) 
            {
                if(i<200 && i>=0) //rate=10s , 1s=>1=10, i600=1m
                {
                    i=i+1;
                    num=1;
                }
                else if(i>=200&&i<400)
                {
                    i=i+1;
                    num=2;
                }
                else
                {
                    i=0;
                    num=1;
                }

            formation_num.data=num;
            num_pub.publish(formation_num);
            }
        
            ros::spinOnce();
            rate.sleep();
        }
    }
};

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "formation_node");

    Formation formation;
    formation.run();

    return 0;
}
            



