#include "publish_initial_pose.hpp"
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

PublishInitialPose::PublishInitialPose(ros::NodeHandle& nh) : nh_(nh) {
    loadRobotPositions();

    // 각 로봇에 대해 개별 퍼블리셔 생성
    for (int i = 1; i <= 5; ++i) {
        ugv_pubs_.emplace_back(nh_.advertise<geometry_msgs::PoseStamped>("/UGV" + std::to_string(i) + "/"  + topic_name_, 10));
        uav_pubs_.emplace_back(nh_.advertise<geometry_msgs::PoseStamped>("/UAV" + std::to_string(i) + "/"  + topic_name_, 10));
    }
}

void PublishInitialPose::loadRobotPositions() {
    ROS_INFO("Loading UGV and UAV positions...");
    
    // UAV 좌표 로드 및 출력
    for (int i = 1; i <= 5; ++i) {
        nh_.getParam("UAV" + std::to_string(i) + "/x", uavs_[i-1].x);
        nh_.getParam("UAV" + std::to_string(i) + "/y", uavs_[i-1].y);
        nh_.getParam("UAV" + std::to_string(i) + "/z", uavs_[i-1].z);
        std::cout << "UAV" << i << " Position Loaded: x=" << uavs_[i-1].x 
                  << ", y=" << uavs_[i-1].y << ", z=" << uavs_[i-1].z << "\n";
    }
    
    std::cout << "\n";

    // UGV 좌표 로드 및 출력
    for (int i = 1; i <= 5; ++i) {
        nh_.getParam("UGV" + std::to_string(i) + "/x", ugvs_[i-1].x);
        nh_.getParam("UGV" + std::to_string(i) + "/y", ugvs_[i-1].y);
        std::cout << "UGV" << i << " Position Loaded: x=" << ugvs_[i-1].x 
                  << ", y=" << ugvs_[i-1].y << ", z=0.0\n";
    }

    nh_.param<std::string>("frame_id", frame_id_);
    nh_.param<std::string>("topic_name", topic_name_, "initial_pose");

}

void PublishInitialPose::publishInitialPose() {
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = ros::Time::now();

    // 각 UGV 로봇의 초기 위치를 해당 토픽으로 발행
    for (int i = 0; i < 5; ++i) {
        pose_msg.header.frame_id = frame_id_;
        pose_msg.pose.position.x = ugvs_[i].x;
        pose_msg.pose.position.y = ugvs_[i].y;
        pose_msg.pose.position.z = 0.0;  // UGV의 Z 좌표는 0
        ugv_pubs_[i].publish(pose_msg);
    }

    // 각 UAV 로봇의 초기 위치를 해당 토픽으로 발행
    for (int i = 0; i < 5; ++i) {
        pose_msg.header.frame_id = frame_id_;
        pose_msg.pose.position.x = uavs_[i].x;
        pose_msg.pose.position.y = uavs_[i].y;
        pose_msg.pose.position.z = uavs_[i].z;
        uav_pubs_[i].publish(pose_msg);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "publish_initial_pose");
    ros::NodeHandle nh;

    PublishInitialPose publisher(nh);
    ros::Rate rate(10);

    while (ros::ok()) {
        publisher.publishInitialPose();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}