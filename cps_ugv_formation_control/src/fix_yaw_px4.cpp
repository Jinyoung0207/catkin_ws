#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include "ros/ros.h"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <mutex>
#include <iostream>
#include <vector>
#include <queue>
#include <memory>
#include <cmath>
#include <chrono>

class PoseCalibrationNode : public rclcpp::Node
{
public:
    PoseCalibrationNode() : Node("pose_calibration_node")
    {
        init();
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33), std::bind(&PoseCalibrationNode::run_timer_, this));
    }
    ~PoseCalibrationNode()
    {}
private:
    ros::NodeHandle nh_;

    void init(){
        std::cout << "Start Cali Node" << std::endl;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);
        subscription_pose_ = nh_.subscribe<geometry_msgs::msg::PoseStamped>("/mavros/local_position/pose", 1, std::bind(&PoseCalibrationNode::px4_callback, this, std::placeholders::_1));
        cali_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStampedz>("/calibrated_pose", 1);
        cali_path_pub = this->create_publisher<nav_msgs::msg::Path>("/cali_path",1);
        initial_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/initial_pose", 1);
        raw_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/raw_pose", 1);
        raw_path_pub = this->create_publisher<nav_msgs::msg::Path>("/raw_path",1);
        rotation_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/rotation_pose", 1);
        rotation_path_pub = this->create_publisher<nav_msgs::msg::Path>("/rotation_path",1);
    }

    void run_timer_(){
        msg_update();
        if(px4_odom_in){
            cali_pose();
            pub_cali_pose();
        }
    }

    void px4_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
    {
        buf.lock();
        px4_odom_queue.push(*msg);
        buf.unlock();
    }

    void msg_update()
    {
        if(!px4_odom_queue.empty())
        {
            buf.lock();
            px4_odom_in = true;
            px4_odom_msgs = px4_odom_queue.back();
            std::queue<px4_msgs::msg::VehicleOdometry> empty_queue;
            px4_odom_queue = empty_queue;
            buf.unlock();
        }
    }

    void cali_pose()
    {
        std::lock_guard<std::mutex> lock(data_mutex_);

        double x = px4_odom_msgs.position[0];
        double y = px4_odom_msgs.position[1];
        double z = px4_odom_msgs.position[2];
        double yaw = get_yaw_from_pose(px4_odom_msgs);

        if (!calibrated_)
        {
            count_++;
            if (count_ > 100 && count_ < 300)
            {
                x_sum_ += x;
                y_sum_ += y;
                z_sum_ += z;
                yaw_sum_ += yaw;
            }
 
            if (count_ == 300)
            {
                initial_x_ = x_sum_ / 200.0;
                initial_y_ = y_sum_ / 200.0;
                initial_z_ = z_sum_ / 200.0;
                initial_yaw_ = yaw_sum_ / 200.0;
                RCLCPP_INFO(this->get_logger(), "Initial pose (x, y, z, yaw) to be zeroed: (%f, %f, %f, %f)", initial_x_, initial_y_, initial_z_, initial_yaw_);
                calibrated_ = true;
            }
        }
        else
        {
            current_x_ = x;
            current_y_ = y;
            current_z_ = z;
            current_yaw_ = yaw;
        }
    }

    void pub_cali_pose()
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (calibrated_)
        {
            double corrected_x = current_x_ - initial_x_;
            double corrected_y = current_y_ - initial_y_;
            double corrected_z = current_z_ - initial_z_;
            double corrected_yaw = current_yaw_ ;//- initial_yaw_;
            if (corrected_yaw < -M_PI)
                corrected_yaw += 2.0 * M_PI;
            if (corrected_yaw > M_PI)
                corrected_yaw -= 2.0 * M_PI;

            // 실시간 raw pose
            geometry_msgs::msg::PoseStamped raw_pose;
            raw_pose.header.stamp = this->now();
            raw_pose.header.frame_id = "map";
            raw_pose.pose.position.x = current_x_;
            raw_pose.pose.position.y = current_y_;
            raw_pose.pose.position.z = current_z_;
            tf2::Quaternion raw_q;
            raw_q.setRPY(0, 0, current_yaw_);
            raw_pose.pose.orientation = tf2::toMsg(raw_q);
            raw_pub->publish(raw_pose);
            // 실시간 raw path
            raw_path_msg.poses.emplace_back(raw_pose);
            raw_path_msg.header.frame_id = "map";
            raw_path_msg.header.stamp = this->now();
            raw_path_pub -> publish(raw_path_msg);

            // 평균 낸 초기 pose
            geometry_msgs::msg::PoseStamped ini_pose;
            ini_pose.header.stamp = this->now();
            ini_pose.header.frame_id = "map";
            ini_pose.pose.position.x = initial_x_;
            ini_pose.pose.position.y = initial_y_;
            ini_pose.pose.position.z = initial_z_;
            tf2::Quaternion ini_q;
            ini_q.setRPY(0, 0, initial_yaw_);
            ini_pose.pose.orientation = tf2::toMsg(ini_q);
            initial_pub->publish(ini_pose);

            // 보정 후 pose
            geometry_msgs::msg::PoseStamped corrected_pose;
            corrected_pose.header.stamp = this->now();
            corrected_pose.header.frame_id = "map";
            corrected_pose.pose.position.x = corrected_x;
            corrected_pose.pose.position.y = corrected_y;
            corrected_pose.pose.position.z = corrected_z;
            tf2::Quaternion corrected_q;
            corrected_q.setRPY(0, 0, corrected_yaw);
            corrected_pose.pose.orientation = tf2::toMsg(corrected_q);
            cali_publisher_->publish(corrected_pose);
            // 보정 후 path
            px4_path_msg.poses.emplace_back(corrected_pose);
            px4_path_msg.header.frame_id = "map";
            px4_path_msg.header.stamp = this->now();
            cali_path_pub -> publish(px4_path_msg);

            // 로테이션 pose
            Eigen::Vector3d original_position(corrected_x, corrected_y, corrected_z);
            Eigen::Quaterniond original_quaternion(corrected_pose.pose.orientation.w, corrected_pose.pose.orientation.x , corrected_pose.pose.orientation.y , corrected_pose.pose.orientation.z);
            Eigen::Quaterniond rotation_quaternion;
            rotation_quaternion = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());
            Eigen::Quaterniond new_quaternion = rotation_quaternion * original_quaternion;
            Eigen::Vector3d rotated_position = rotation_quaternion * original_position;

            geometry_msgs::msg::PoseStamped eigen_pose_msg;
            eigen_pose_msg.header.stamp = this->now();
            eigen_pose_msg.header.frame_id = "map";
            eigen_pose_msg.pose.position.x = rotated_position.x();
            eigen_pose_msg.pose.position.y = rotated_position.y();
            eigen_pose_msg.pose.position.z = rotated_position.z();
            eigen_pose_msg.pose.orientation.w = new_quaternion.w();
            eigen_pose_msg.pose.orientation.x = new_quaternion.x();
            eigen_pose_msg.pose.orientation.y = new_quaternion.y();
            eigen_pose_msg.pose.orientation.z = new_quaternion.z();
            rotation_publisher_->publish(eigen_pose_msg);
            // 로테이션 path
            eigen_path_msg.poses.emplace_back(eigen_pose_msg);
            eigen_path_msg.header.frame_id = "map";
            eigen_path_msg.header.stamp = this->now();
            rotation_path_pub -> publish(eigen_path_msg);

            std::cout << "Original position: " << original_position.transpose() << std::endl;
            std::cout << "Rotated position: " << rotated_position.transpose() << std::endl;
            RCLCPP_INFO(this->get_logger(), "Corrected pose: x=%f, y=%f, z=%f, yaw=%f", corrected_x, corrected_y, corrected_z, corrected_yaw);
            std::cout << "==============================================" << std::endl;
        }
    }

    double get_yaw_from_pose(const px4_msgs::msg::VehicleOdometry &pose)
    {
        // 쿼터니언 변환
        tf2::Quaternion q(
            pose.q[1],
            pose.q[2],
            pose.q[3],
            pose.q[0]
        );
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        return yaw;
    }

    // 변환전 PX4_msg 관련
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr subscription_pose_;
    px4_msgs::msg::VehicleOdometry px4_odom_msgs;
    std::queue<px4_msgs::msg::VehicleOdometry> px4_odom_queue;

    // 변환된 PX4_msg 관련
    nav_msgs::msg::Path raw_path_msg;
    nav_msgs::msg::Path px4_path_msg;
    nav_msgs::msg::Path eigen_path_msg;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr cali_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr rotation_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr initial_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr raw_pub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr cali_path_pub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr raw_path_pub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr rotation_path_pub;

    // 기타
    rclcpp::TimerBase::SharedPtr timer_;
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    std::mutex buf;
    std::mutex data_mutex_;
    size_t count_;
    double x_sum_, y_sum_, z_sum_, yaw_sum_;
    double initial_x_, initial_y_, initial_z_, initial_yaw_;
    double current_x_, current_y_, current_z_, current_yaw_;
    bool calibrated_ = false;
    bool px4_odom_in = false;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_calibration_node");
    rclcpp::spin(std::make_shared<PoseCalibrationNode>());
    rclcpp::shutdown();
    return 0;
}