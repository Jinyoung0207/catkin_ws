#pragma once

#include <ros/ros.h>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float32.h>

#include <iostream>
#include <cmath>
#include <vector>
#include <string>

#include <termios.h>
#include <stdio.h>

#define   END    "\033[0m"
#define   RED    "\033[1;31m"
#define   GREEN  "\033[1;32m"
#define   BLUE   "\033[1;34m"
#define   CYAN   "\033[1;36m"
#define   YELL   "\033[1;33m"

class formation_order
{
public:
    formation_order(ros::NodeHandle& nh)
    {}
    ~formation_order()
    {}
    void param_read();
    void init();
    void run();
private:
    ros::NodeHandle nh;

    ros::Publisher uav_formation_pub;
    ros::Publisher ugv_formation_pub;


    ros::Publisher th_pub;
    ros::Publisher land_vel_pub;

    ros::Subscriber UAV1_init_pose_sub;
    ros::Subscriber UAV2_init_pose_sub;
    ros::Subscriber UAV3_init_pose_sub;
    ros::Subscriber UAV4_init_pose_sub;
    ros::Subscriber UAV5_init_pose_sub;

    ros::Subscriber UGV1_init_pose_sub;
    ros::Subscriber UGV2_init_pose_sub;
    ros::Subscriber UGV3_init_pose_sub;
    ros::Subscriber UGV4_init_pose_sub;
    ros::Subscriber UGV5_init_pose_sub;


    ros::Publisher formation_num_pub;
    std_msgs::Int64 f_num;
    std_msgs::Float32 th_;
    std_msgs::Float32 land_vel_;

    geometry_msgs::PoseArray uav_formation_msg;
    geometry_msgs::PoseArray ugv_formation_msg;

    std::map<std::string,std::vector<geometry_msgs::Point>> uav_formation_list;
    std::map<std::string,std::vector<geometry_msgs::Point>> ugv_formation_list;

    std::vector<geometry_msgs::Point> uav_order_formation;
    std::vector<geometry_msgs::Point> ugv_order_formation;

    std::vector<geometry_msgs::Point> uav_prev_formation;
    std::vector<geometry_msgs::Point> ugv_prev_formation;

    std::vector<std::vector<geometry_msgs::Point>> uav_change_formation_vec;
    std::vector<std::vector<geometry_msgs::Point>> ugv_change_formation_vec;

    int loop_count = 0;
    int wait_time=0;

    int choose_set_input = 0;
    int uav_formation_input = 0;
    int ugv_formation_input = 0;
    int all_formation_input = 0;

    bool set_key_input_in = false;
    bool set_fir_key_input_in = false;

    bool uav_key_input_in = false;
    bool uav_fir_key_input_in = false;

    bool ugv_key_input_in = false;
    bool ugv_fir_key_input_in = false;

    bool all_key_input_in = false;
    bool all_fir_key_input_in = false;
    
    bool formation_order_end = false;
    bool formation_set_end = false;
    bool formation_sampling = false;
    bool formation_changing = false;
    bool formation_change_end= false;
    
    bool formation_order_end_ugv = false;
    bool formation_set_end_ugv = false;
    bool formation_sampling_ugv = false;
    bool formation_changing_ugv = false;
    bool formation_change_end_ugv= false;


    double scale_nor = 312.0;
    double set_vel = 0.1;
    double rate = 5;
    double dt = 0;
    double uav_z_max = 1.0;
    double scale;
    
    int move=0;

    // Initial offset
    double init_ugv1_x;
    double init_ugv1_y;
    double init_ugv2_x;
    double init_ugv2_y;
    double init_ugv3_x;
    double init_ugv3_y;
    double init_ugv4_x;
    double init_ugv4_y;
    double init_ugv5_x;
    double init_ugv5_y;
    
    bool uav1_on=true;
    bool uav2_on=true;
    bool uav3_on=true;
    bool uav4_on=true;
    bool uav5_on=true;

    bool ugv1_on=true;
    bool ugv2_on=true;
    bool ugv3_on=true;
    bool ugv4_on=true;
    bool ugv5_on=true;
    
    double d12=0;
    double d13=0;
    double d14=0;
    double d15=0;
    
    double d12_ugv=0;
    double d13_ugv=0;
    double d14_ugv=0;
    double d15_ugv=0;
    
    geometry_msgs::PoseStamped init_uav1;
    geometry_msgs::PoseStamped init_uav2;
    geometry_msgs::PoseStamped init_uav3;
    geometry_msgs::PoseStamped init_uav4;
    geometry_msgs::PoseStamped init_uav5;

    geometry_msgs::PoseStamped init_ugv1;
    geometry_msgs::PoseStamped init_ugv2;
    geometry_msgs::PoseStamped init_ugv3;
    geometry_msgs::PoseStamped init_ugv4;
    geometry_msgs::PoseStamped init_ugv5;


    double inital_x1;
    double inital_y1;
    double inital_x2;
    double inital_y2;
    double inital_x3;
    double inital_y3;
    double inital_x4;
    double inital_y4;
    double inital_x5;
    double inital_y5;

    //formation offset
    double form1_ugv1_x;
    double form1_ugv1_y;
    double form1_ugv2_x;
    double form1_ugv2_y;
    double form1_ugv3_x;
    double form1_ugv3_y;
    double form1_ugv4_x;
    double form1_ugv4_y;
    double form1_ugv5_x;
    double form1_ugv5_y;
    double form2_ugv1_x;
    double form2_ugv1_y;
    double form2_ugv2_x;
    double form2_ugv2_y;
    double form2_ugv3_x;
    double form2_ugv3_y;
    double form2_ugv4_x;
    double form2_ugv4_y;
    double form2_ugv5_x;
    double form2_ugv5_y;
    double form3_ugv1_x;
    double form3_ugv1_y;
    double form3_ugv2_x;
    double form3_ugv2_y;
    double form3_ugv3_x;
    double form3_ugv3_y;
    double form3_ugv4_x;
    double form3_ugv4_y;
    double form3_ugv5_x;
    double form3_ugv5_y;
    
    double form4_ugv1_x;
    double form4_ugv1_y;
    double form4_ugv2_x;
    double form4_ugv2_y;
    double form4_ugv3_x;
    double form4_ugv3_y;
    double form4_ugv4_x;
    double form4_ugv4_y;
    double form4_ugv5_x;
    double form4_ugv5_y;
    
    double form1_uav1_x;
    double form1_uav1_y;
    double form1_uav1_z;
    double form1_uav2_x;
    double form1_uav2_y;
    double form1_uav2_z;
    double form1_uav3_x;
    double form1_uav3_y;
    double form1_uav3_z;
    double form1_uav4_x;
    double form1_uav4_y;
    double form1_uav4_z;
    double form1_uav5_x;
    double form1_uav5_y;
    double form1_uav5_z;

    double form2_uav1_x;
    double form2_uav1_y;
    double form2_uav1_z;
    double form2_uav2_x;
    double form2_uav2_y;
    double form2_uav2_z;
    double form2_uav3_x;
    double form2_uav3_y;
    double form2_uav3_z;
    double form2_uav4_x;
    double form2_uav4_y;
    double form2_uav4_z;
    double form2_uav5_x;
    double form2_uav5_y;
    double form2_uav5_z;

    double form3_uav1_x;
    double form3_uav1_y;
    double form3_uav1_z;
    double form3_uav2_x;
    double form3_uav2_y;
    double form3_uav2_z;
    double form3_uav3_x;
    double form3_uav3_y;
    double form3_uav3_z;
    double form3_uav4_x;
    double form3_uav4_y;
    double form3_uav4_z;
    double form3_uav5_x;
    double form3_uav5_y;
    double form3_uav5_z;

    double form4_uav1_x;
    double form4_uav1_y;
    double form4_uav1_z;
    double form4_uav2_x;
    double form4_uav2_y;
    double form4_uav2_z;
    double form4_uav3_x;
    double form4_uav3_y;
    double form4_uav3_z;
    double form4_uav4_x;
    double form4_uav4_y;
    double form4_uav4_z;
    double form4_uav5_x;
    double form4_uav5_y;
    double form4_uav5_z;

    std::string frame_id = "world";

    int uav_formation_pattern = 0;
    int ugv_formation_pattern = 0;

    int uav_changing_point_id_max = 0;
    int ugv_changing_point_id_max = 0;

    int uav_changing_point_id = 0;
    int ugv_changing_point_id = 0;

    std::vector<int> uav_assignment;
    std::vector<int> ugv_assignment;

    void UAV1_init_pose_Callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void UAV2_init_pose_Callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void UAV3_init_pose_Callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void UAV4_init_pose_Callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void UAV5_init_pose_Callback(const geometry_msgs::PoseStamped::ConstPtr& msg);

    
    void UGV1_init_pose_Callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void UGV2_init_pose_Callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void UGV3_init_pose_Callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void UGV4_init_pose_Callback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void UGV5_init_pose_Callback(const geometry_msgs::PoseStamped::ConstPtr& msg);

    void formation_init_set();    
    void key_read();
    void input_set();
    void formation_pattern();
    void formation_sampling_point();
    void formation_changing_point();
    void formation_reset();
    void msg_set();
    void publish();
    void initial_pose_pub();
    void autolanding_pub();
    void reset();
};

