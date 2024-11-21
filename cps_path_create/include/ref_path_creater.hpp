#include <ros/ros.h>

#include <nav_msgs/Path.h>
#include <std_msgs/Float64MultiArray.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point.h>

#include <queue>
#include <mutex>
#include <tf/tf.h>
#include <cmath>
#include <iostream>

#define   END    "\033[0m"
#define   RED    "\033[1;31m"
#define   GREEN  "\033[1;32m"
#define   BLUE   "\033[1;34m"
#define   CYAN   "\033[1;36m"
#define   YELL   "\033[1;33m"

class ref_path_cps
{
public:
    ref_path_cps(ros::NodeHandle& nh)
    {}
    ~ref_path_cps()
    {}

    void read_param();
    void init();
    void run();
    void set_init_formation();

private:
    // ROS 관련 멤버 변수
    ros::NodeHandle node_handle_;
    
    ros::Subscriber formation_sub_;
    ros::Subscriber global_path_sub_;
    ros::Subscriber ugv_formation_sub;
    ros::Subscriber uav_formation_sub;

    ros::Publisher lead_ugv_ref_traj_pub;
    ros::Publisher f1_ugv_ref_traj_pub;
    ros::Publisher f2_ugv_ref_traj_pub;
    ros::Publisher f3_ugv_ref_traj_pub;
    ros::Publisher f4_ugv_ref_traj_pub;

    ros::Publisher lead_uav_ref_traj_pub;
    ros::Publisher f1_uav_ref_traj_pub;
    ros::Publisher f2_uav_ref_traj_pub;
    ros::Publisher f3_uav_ref_traj_pub;
    ros::Publisher f4_uav_ref_traj_pub;

    ros::Publisher lead_uav_point_pub;
    ros::Publisher f1_uav_point_pub;
    ros::Publisher f2_uav_point_pub;
    ros::Publisher f3_uav_point_pub;
    ros::Publisher f4_uav_point_pub;

    // 추가적인 publisher들

    // Buffer와 플래그
    std::queue<nav_msgs::Path> global_path_queue_;
    std::mutex buf_;
    bool global_path_in_= false;
    bool create_ref_end_= false;

    // 파라미터
    double dt_;
    int node;
    double v_mean_;
    double x_max_;
    double rate_;
    double curve_y_max_;

    // Formation offsets
    double ld_x_ = 0.0;
    double ld_y_ = 0.0;
    double f1_x_ = 0.0;
    double f1_y_ = -3.0;
    double f2_x_ = 0.0;
    double f2_y_ = 3.0;
    double f3_x_ = 0.0;
    double f3_y_ = -6.0;
    double f4_x_ = 0.0;
    double f4_y_ = 6.0;

    double ld_x_uav = 0.0;
    double ld_y_uav = 0.0;
    double ld_z_uav = 2.0;
    double f1_x_uav = 0.0;
    double f1_y_uav = -3.0;
    double f1_z_uav = 2.0;
    double f2_x_uav = 0.0;
    double f2_y_uav = 3.0;
    double f2_z_uav = 2.0;
    double f3_x_uav = 0.0;
    double f3_y_uav = -6.0;
    double f3_z_uav = 2.0;
    double f4_x_uav = 0.0;
    double f4_y_uav = 6.0;
    double f4_z_uav = 2.0;

    // 키 입력 관련 플래그
    int key_result_;
    bool key_board_in_;
    bool fir_key_input_in_;

    double scale_nor = 1.0;

    // 기타 변수들
    std::string global_path_topic_ = "";
    //ugv
    std::string lead_ugv_ref_traj_topic = "";
    std::string f1_ugv_ref_traj_topic = "";
    std::string f2_ugv_ref_traj_topic = "";
    std::string f3_ugv_ref_traj_topic = "";
    std::string f4_ugv_ref_traj_topic = "";
    //uav
    std::string lead_uav_ref_traj_topic = "";
    std::string f1_uav_ref_traj_topic = "";
    std::string f2_uav_ref_traj_topic = "";
    std::string f3_uav_ref_traj_topic = "";
    std::string f4_uav_ref_traj_topic = "";

    std::string frame_id_ = "";

    tf::Quaternion last_head;
    int make_ref_id_ = 0;
    int path_loop_count = 0;

    // 메시지 타입
    nav_msgs::Path global_path_msg_;
    nav_msgs::Path lead_ugv_ref_traj_msg;
    nav_msgs::Path f1_ugv_ref_traj_msg;
    nav_msgs::Path f2_ugv_ref_traj_msg;
    nav_msgs::Path f3_ugv_ref_traj_msg;
    nav_msgs::Path f4_ugv_ref_traj_msg;
    nav_msgs::Path lead_uav_ref_traj_msg;
    nav_msgs::Path f1_uav_ref_traj_msg;
    nav_msgs::Path f2_uav_ref_traj_msg;
    nav_msgs::Path f3_uav_ref_traj_msg;
    nav_msgs::Path f4_uav_ref_traj_msg;
    nav_msgs::Path formation_ref_path_msg;
    nav_msgs::Path formation_uav_ref_path_msg;
    geometry_msgs::PoseStamped uav0_point_msg;
    geometry_msgs::PoseStamped uav1_point_msg;
    geometry_msgs::PoseStamped uav2_point_msg;
    geometry_msgs::PoseStamped uav3_point_msg;
    geometry_msgs::PoseStamped uav4_point_msg;

    // 콜백 함수들
    // void formation_Callback(const std_msgs::Float64MultiArray::ConstPtr& msg);
    void global_path_Callback(const nav_msgs::Path::ConstPtr& msg);
    void ugv_formation_Callback(const geometry_msgs::PoseArray::ConstPtr& msg);
    void uav_formation_Callback(const geometry_msgs::PoseArray::ConstPtr& msg);

    // 유틸리티 함수들
    void global_path_update();
    void make_formation_ref_trajectory();
    void make_ref_trajectory();
    void publish();
    void reset();
    
};
