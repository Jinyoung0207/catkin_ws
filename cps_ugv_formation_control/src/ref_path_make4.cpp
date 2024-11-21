#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Float64.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"
#include <memory>
#include <cmath>
#include <iostream>
#include <vector>

class ref_path_make4 {
public:
    ref_path_make4() {
        ros::NodeHandle nh;

        global_path_publisher_ = nh.advertise<nav_msgs::Path>("path4", 10);
        ref_path_publisher_ = nh.advertise<nav_msgs::Path>("/rbt4/ref_trajectory", 10);
        subscription_ = nh.subscribe("/global_path4", 10, &ref_path_make4::pathCallback, this);

        global_timer_ = nh.createTimer(ros::Duration(0.1), &ref_path_make4::globalPathMake, this);
        ref_path_timer_ = nh.createTimer(ros::Duration(0.1), &ref_path_make4::refPathMake, this);
    }

private:
    ros::Publisher global_path_publisher_;
    ros::Publisher ref_path_publisher_;
    ros::Subscriber subscription_;
    ros::Timer ref_path_timer_;
    ros::Timer global_timer_;
    std::vector<std::pair<double, double>> Astar_poses;
    std::vector<std::pair<double, double>> Astar_result;

    double dt = 0.1;
    int N = 30;
    double ref_path_speed = 0.2;
    int ref_traj_start = 0;
    double global_path_len = 0;
    int global_node_count = 0;
    double node_len = 0;
    double yaw = 0;
    int next_num = 0;
    int Astar_poses_size = 0;
    double Astar_grid_size = 0.5;
    double Astar_ref_node_count_all = 0;
    int test = 0;

    double roundToThirdDecimalPlace(double value) {
        return std::round(value * 1000.0) / 1000.0;
    }

    void pathCallback(const nav_msgs::Path::ConstPtr& msg) {
        Astar_poses_size = msg->poses.size();

        for (const auto &pose_stamped : msg->poses) {
            double x = pose_stamped.pose.position.x;
            double y = pose_stamped.pose.position.y;

            if (Astar_poses.size() < Astar_poses_size) {
                Astar_poses.emplace_back(std::make_pair(x, y));
            }
        }

        global_path_len = double(Astar_grid_size * (Astar_poses_size - 2.0));
        global_node_count = int(global_path_len / (ref_path_speed * dt));
        node_len = global_path_len / double(global_node_count);
        node_len = roundToThirdDecimalPlace(node_len);
        //std::cout << node_len << " node len" << std::endl;

        if ((Astar_poses.size() == Astar_poses_size) && (test == 0)) {
            makeAstarVector();
            test += 1;
        }
    }

    void makeAstarVector() {
        std::pair<double, double> prev_point = Astar_poses[0];
        double prev_x = roundToThirdDecimalPlace(prev_point.first);
        double prev_y = roundToThirdDecimalPlace(prev_point.second);
        double fractionalPart = 0.0;

        for (size_t i = 1; i < int(Astar_poses.size()); ++i) {
            std::pair<double, double> curr_point = Astar_poses[i];
            double curr_x = roundToThirdDecimalPlace(curr_point.first);
            double curr_y = roundToThirdDecimalPlace(curr_point.second);
            double new_x = 0.0;
            double new_y = 0.0;

            double distance = std::abs(sqrt(pow(curr_x - prev_x, 2) + pow(curr_y - prev_y, 2)));
            Astar_ref_node_count_all = roundToThirdDecimalPlace((distance / node_len) + fractionalPart);
            int integerPart = static_cast<int>(Astar_ref_node_count_all);
            fractionalPart = Astar_ref_node_count_all - integerPart;

            for (int j = 0; j < integerPart; ++j) {
                if (curr_x >= prev_x) {
                    new_x = (std::abs(curr_x - prev_x) < node_len) ? prev_x : prev_x + node_len;
                } else {
                    new_x = prev_x - node_len;
                }
                if (curr_y >= prev_y) {
                    new_y = (std::abs(curr_y - prev_y) < node_len) ? prev_y : prev_y + node_len;
                } else {
                    new_y = prev_y - node_len;
                }
                if (prev_x != new_x || prev_y != new_y) {
                    Astar_result.emplace_back(std::make_pair(new_x, new_y));
                }
                prev_x = new_x;
                prev_y = new_y;
            }
        }
    }

    void globalPathMake(const ros::TimerEvent&) {
        nav_msgs::Path global_path_msg;
        global_path_msg.header.frame_id = "world";
        global_path_msg.header.stamp = ros::Time::now();

        for (const auto &pos : Astar_poses) {
            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.header.stamp = global_path_msg.header.stamp;
            pose_stamped.header.frame_id = "world";
            pose_stamped.pose.position.x = pos.first;
            pose_stamped.pose.position.y = pos.second;
            global_path_msg.poses.emplace_back(pose_stamped);
        }
        global_path_publisher_.publish(global_path_msg);
    }

    void refPathMake(const ros::TimerEvent&) {
        nav_msgs::Path ref_path_msg;
        ref_path_msg.header.frame_id = "world";
        ref_path_msg.header.stamp = ros::Time::now();

        if (test >= 1) {
            for (int i = 0; i < N; ++i) {
                double current_x = Astar_result[ref_traj_start + i].first;
                double current_y = Astar_result[ref_traj_start + i].second;

                if (i < N) {
                    next_num = (ref_traj_start + i == int(Astar_poses.size()) - 1) ? 0 : 1;
                } else {
                    next_num = 0;
                }

                double next_x = Astar_result[ref_traj_start + i + next_num].first;
                double next_y = Astar_result[ref_traj_start + i + next_num].second;

                refPathYaw(current_x, current_y, next_x, next_y);
                geometry_msgs::PoseStamped pose_stamped;
                pose_stamped.header.stamp = ref_path_msg.header.stamp;
                pose_stamped.header.frame_id = "world";
                pose_stamped.pose.position.x = current_x;
                pose_stamped.pose.position.y = current_y;
                tf::Quaternion quat;
                quat.setRPY(0, 0, yaw);
                pose_stamped.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

                ref_path_msg.poses.emplace_back(pose_stamped);
            }

            geometry_msgs::PoseStamped first_pose = ref_path_msg.poses[0];
            double first_x = first_pose.pose.position.x;
            double first_y = first_pose.pose.position.y;
            //ROS_INFO(" First _ X : %f", first_x);
            //ROS_INFO(" First _ Y : %f", first_y);
            //ROS_INFO(" ============ ");
            ref_path_publisher_.publish(ref_path_msg);
            updatePath();
        }
    }

    int updatePath() {
        if (ref_traj_start < (Astar_result.size() - 1)) {
            ref_traj_start += 1;
            if (ref_traj_start >= (int(Astar_result.size()) - N - 1)) {
                N = (N <= 1) ? 1 : N - 1;
            }
        }
        return ref_traj_start;
    }

    double piToPi(double angles) {
        while (angles >= M_PI)
            angles -= 2.0 * M_PI;
        while (angles < -M_PI)
            angles += 2.0 * M_PI;
        return angles;
    }

    double refPathYaw(double current_x, double current_y, double next_x, double next_y) {
        if (current_x == next_x && current_y == next_y) {
            yaw = yaw;
        } else {
            double delta_x = next_x - current_x;
            double delta_y = next_y - current_y;
            yaw = atan2(delta_y, delta_x);
            yaw = piToPi(yaw);
        }
        return yaw;
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "ref_path_make4");
    ref_path_make4 make_traj05;
    ros::spin();
    return 0;
}