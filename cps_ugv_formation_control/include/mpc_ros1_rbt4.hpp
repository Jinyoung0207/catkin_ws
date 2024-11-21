#pragma once

#include <iostream>
#include <vector>
#include <queue>
#include <string>
#include <cmath>
#include <mutex>
#include <memory>
#include <functional>

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Point.h"

#include "std_msgs/ColorRGBA.h"

#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"

#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

#include "tf/LinearMath/Quaternion.h"
#include "tf/LinearMath/Matrix3x3.h"

#include "ros/ros.h"

#include <casadi/casadi.hpp>


#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

using namespace std;

class mpc_ros1_rbt4 : public ros::NodeHandle
{
public:
    /*mpc_ros1_rbt4() : NodeHandle("mpc_ros1_rbt4")
    {

    }
    ~mpc_ros1_rbt4()
    {}*/

    void node_param_init();
    void init();
    void run();

private:
    // ROS1
    ros::Time ros1time;
    ros::NodeHandle nh_;

    ros::Subscriber rbt_odom_sub;
    ros::Subscriber rbt_ref_traj_sub;

    ros::Publisher rbt_cmd_pub;
    ros::Publisher rbt_pre_pub;
    ros::Publisher marker_pub;

    ros::Subscriber rbt1_path_sub;
    nav_msgs::Path rbt1_path_msg;
    queue<nav_msgs::Path> rbt1_path_queue;
    ros::Subscriber rbt2_path_sub;
    nav_msgs::Path rbt2_path_msg;
    queue<nav_msgs::Path> rbt2_path_queue;
    ros::Subscriber rbt3_path_sub;
    nav_msgs::Path rbt3_path_msg;
    queue<nav_msgs::Path> rbt3_path_queue;

    nav_msgs::Odometry rbt_odom_msg;
    nav_msgs::Path rbt_ref_traj_msg;
    geometry_msgs::Twist rbt_cmd_msg;
    nav_msgs::Path rbt_pre_msg;
    nav_msgs::Path rbt_ref_msg;

    queue<nav_msgs::Odometry> rbt_odom_queue;
    queue<nav_msgs::Path> rbt_ref_traj_queue;
    mutex buf;

    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker marker;

    bool rbt_odom_in = false;
    bool rbt_ref_traj_in = false;
    bool sub_print_once = false;
    bool rbt1_pre_in = false;
    bool rbt2_pre_in = false;
    bool rbt3_pre_in = false;

    string ref_topic;
    string odom_topic;
    string cmd_topic;
    string pre_topic;
    string map_id;

    // MPC 
    double dt;
    int N;
    int pre_path_N = 30;
    int robot_n = 3;

    casadi::SX x;
    casadi::SX y;
    casadi::SX theta;
    casadi::SX states;
    int n_states;

    casadi::SX v;
    casadi::SX omega;
    casadi::SX controls;
    int n_controls;

    casadi::SX rhs;

    double Q_x     = 0.1;
    double Q_y     = 0.1;
    double Q_theta = 0.01;

    double Q_terminor = 10.0;

    double R1 = 0.005;
    double R2 = 0.005;

    casadi::Function solver;
    casadi::Function f;
    casadi::SX U;
    casadi::SX X;
    casadi::SX P;

    casadi::SX Q;
    casadi::SX R;
    casadi::SX Q_ter;

    casadi::SX g;

    double x_max     =  casadi::inf;
    double x_min     = -casadi::inf;
    double y_max     =  casadi::inf;
    double y_min     = -casadi::inf;
    double theta_max =  casadi::inf;
    double theta_min = -casadi::inf;

    // double v_max     = 0.3;
    double v_max     = 0.0;
    double v_min     = 0.0;
    // double omega_max = M_PI/4.0;
    // double omega_min = -(M_PI/4.0);
    double omega_max = M_PI/4.0;
    double omega_min = -(M_PI/4.0);

    casadi::DM lbg;
    casadi::DM ubg;
    casadi::DM lbx;
    casadi::DM ubx;

    casadi::DMDict args;

    casadi::DM rbt_state_init;
    casadi::DM X0;
    casadi::DM U0;

    casadi::DMDict solver_result;

    int loop_count = 0;
    vector<vector<double>> obstacle_vec;
    int n_obstacles = 0;
    // loop
    void rbt_odom_Callbck(const nav_msgs::Odometry::ConstPtr& msg);
    void rbt_ref_traj_Callbck(const nav_msgs::Path::ConstPtr& msg);
    void rbt_odom_update();
    void rbt_ref_traj_update();
    void state_update();
    void setting_solver();
    void make_args_p();
    void setting_reference();
    void reshape_and_init_opt_variable();
    void call_solver();
    void get_result();
    void current_pos_msg_set();
    void cmd_vel_msg_set();
    void predictive_traj_msg_set();
    void publish();
    void shift();
    void reset();

    void avoid_obstacle();
    // once
    void rbt1_path_Callback(const nav_msgs::Path::ConstPtr& msg);
    void rbt1_path_update();
    void rbt2_path_Callback(const nav_msgs::Path::ConstPtr& msg);
    void rbt2_path_update();
    void rbt3_path_Callback(const nav_msgs::Path::ConstPtr& msg);
    void rbt3_path_update();
    void mpc_init();
};