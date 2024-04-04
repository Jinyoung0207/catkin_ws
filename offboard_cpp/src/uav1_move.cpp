#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Path.h> //path

class OffboardController 
{
private:
    ros::NodeHandle nh_;
    ros::Publisher set_point_pub;

    ros::Subscriber state_sub;
    ros::Subscriber ref_path_sub;
    ros::Subscriber current_pose_sub;

    ros::ServiceClient arming_client;
    ros::ServiceClient set_mode_client;

    mavros_msgs::State current_state;
    nav_msgs::Path ref_path_msg;
    geometry_msgs::PoseStamped current_pose_msg;

  
    void stateCallback(const mavros_msgs::State::ConstPtr& msg) 
    {
        current_state = *msg;
    }
    void ref_pathCallback(const nav_msgs::Path::ConstPtr& msg) 
    {
        ref_path_msg = *msg;
    }
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) 
    {
        current_pose_msg = *msg;
    }

    
public:
    OffboardController() : nh_(""),current_state(),ref_path_msg()
    {
        state_sub = nh_.subscribe<mavros_msgs::State>("uav1/mavros/state", 10, &OffboardController::stateCallback, this);
        arming_client = nh_.serviceClient<mavros_msgs::CommandBool>("uav1/mavros/cmd/arming");
        set_mode_client= nh_.serviceClient<mavros_msgs::SetMode>("uav1/mavros/set_mode");
        ref_path_sub = nh_.subscribe<nav_msgs::Path>("uav1/ref_path", 10, &OffboardController::ref_pathCallback, this);
        current_pose_sub = nh_.subscribe<geometry_msgs::PoseStamped>("uav0/mavros/local_position/pose", 10, &OffboardController::poseCallback, this);


        set_point_pub = nh_.advertise<geometry_msgs::PoseStamped>("uav1/mavros/setpoint_position/local", 10);
    }

    void run() 
    {
        ros::Rate rate(20.0);
        geometry_msgs::PoseStamped pose1;

        pose1.pose.position.x = 0;
        pose1.pose.position.y = 0;
        pose1.pose.position.z = 2;

         // Send a few setpoints before starting
        for (int i = 100; ros::ok() && i > 0; --i) 
        {
            set_point_pub.publish(pose1);
            ros::spinOnce();
            rate.sleep();
        }

        // Wait for FCU connection
        while (ros::ok() && !current_state.connected) 
        {
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
                    ROS_INFO("uav1 Offboard enabled");
                }
                last_request = ros::Time::now();
            } 
            else 
            {
                if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))) 
                {
                    if (arming_client.call(arm_cmd) && arm_cmd.response.success) 
                    {
                        ROS_INFO("uav1 armed");
                    }
                    last_request = ros::Time::now();
                }
            }
                //ref_path_msg.header.frame_id = "map";
            pose1.pose.position.x = ref_path_msg.poses[0].pose.position.x+5;
            pose1.pose.position.y = ref_path_msg.poses[0].pose.position.y-5;
            pose1.pose.position.z = ref_path_msg.poses[0].pose.position.z;
            
            set_point_pub.publish(pose1);

            ros::spinOnce();
            rate.sleep();
        }
    }
};

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "uav1_move_node");

    OffboardController offboard_controller;
    offboard_controller.run();

    return 0;
}