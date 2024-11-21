#include <ros/ros.h>

#include <nav_msgs/Path.h>
#include <std_msgs/Float64MultiArray.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float32.h>

#include <std_msgs/Int64.h>
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
    ros::Subscriber UAV1_alt_check_sub;
    ros::Subscriber UAV2_alt_check_sub;
    ros::Subscriber UAV3_alt_check_sub;
    ros::Subscriber UAV4_alt_check_sub;
    ros::Subscriber UAV5_alt_check_sub;

    ros::Subscriber UAV1_error_sub;
    ros::Subscriber UAV2_error_sub;
    ros::Subscriber UAV3_error_sub;
    ros::Subscriber UAV4_error_sub;
    ros::Subscriber UAV5_error_sub;

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

    //initial formation offset
  
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
    
    // Formation offsets
    double ld_x_;
    double ld_y_;
    double f1_x_;
    double f1_y_;
    double f2_x_;
    double f2_y_;
    double f3_x_;
    double f3_y_;
    double f4_x_;
    double f4_y_;

    double ld_x_uav;
    double ld_y_uav;
    double ld_z_uav;
    double f1_x_uav;
    double f1_y_uav;
    double f1_z_uav;
    double f2_x_uav;
    double f2_y_uav;
    double f2_z_uav;
    double f3_x_uav;
    double f3_y_uav;
    double f3_z_uav;
    double f4_x_uav;
    double f4_y_uav;
    double f4_z_uav;

    // 키 입력 관련 플래그
    int key_result_;
    bool key_board_in_;
    bool fir_key_input_in_;

    double scale_nor = 1.0;

    double tolerance;

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

    std::string check_uav = "";

    

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
    std_msgs::Int64 UAV1_alt;
    std_msgs::Int64 UAV2_alt;
    std_msgs::Int64 UAV3_alt;
    std_msgs::Int64 UAV4_alt;
    std_msgs::Int64 UAV5_alt;
    std_msgs::Float32 UAV1_error;
    std_msgs::Float32 UAV2_error;
    std_msgs::Float32 UAV3_error;
    std_msgs::Float32 UAV4_error;
    std_msgs::Float32 UAV5_error;

    // 콜백 함수들
    // void formation_Callback(const std_msgs::Float64MultiArray::ConstPtr& msg);
    void global_path_Callback(const nav_msgs::Path::ConstPtr& msg);
    void ugv_formation_Callback(const geometry_msgs::PoseArray::ConstPtr& msg);
    void uav_formation_Callback(const geometry_msgs::PoseArray::ConstPtr& msg);
    void UAV1_alt_Callback(const std_msgs::Int64::ConstPtr&msg);
    void UAV2_alt_Callback(const std_msgs::Int64::ConstPtr&msg);
    void UAV3_alt_Callback(const std_msgs::Int64::ConstPtr&msg);
    void UAV4_alt_Callback(const std_msgs::Int64::ConstPtr&msg);
    void UAV5_alt_Callback(const std_msgs::Int64::ConstPtr&msg);
    void UAV1_error_Callback(const std_msgs::Float32::ConstPtr&msg);
    void UAV2_error_Callback(const std_msgs::Float32::ConstPtr&msg);
    void UAV3_error_Callback(const std_msgs::Float32::ConstPtr&msg);
    void UAV4_error_Callback(const std_msgs::Float32::ConstPtr&msg);
    void UAV5_error_Callback(const std_msgs::Float32::ConstPtr&msg);

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


    // 유틸리티 함수들
    void global_path_update();
    void make_formation_ref_trajectory();
    void make_ref_trajectory();
    void publish();
    void reset();
    
};
