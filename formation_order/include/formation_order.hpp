#pragma once

#include <ros/ros.h>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point.h>

#include <iostream>
#include <cmath>
#include <vector>
#include <string>

#include <termios.h>
#include <stdio.h>

#include "Hungarian.hpp"

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
    bool formation_change_end = false;

    double scale_nor = 312.0;
    double set_vel = 0.1;
    double rate = 5;
    double dt = 0;
    double uav_z_max = 1.0;
    std::string frame_id = "world";

    int uav_formation_pattern = 0;
    int ugv_formation_pattern = 0;

    int uav_changing_point_id_max = 0;
    int ugv_changing_point_id_max = 0;

    int uav_changing_point_id = 0;
    int ugv_changing_point_id = 0;

    std::vector<int> uav_assignment;
    std::vector<int> ugv_assignment;

    void formation_init_set();
    
    void key_read();
    void input_set();
    void formation_hungarian();
    void formation_sampling_point();
    void formation_changing_point();
    void formation_reset();
    void msg_set();
    void publish();
    void reset();
};

