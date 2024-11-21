#ifndef PUBLISH_INITIAL_POSE_HPP
#define PUBLISH_INITIAL_POSE_HPP

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>

struct Position {
    double x;
    double y;
    double z;
};

class PublishInitialPose {
public:
    PublishInitialPose(ros::NodeHandle& nh);
    void publishInitialPose();

private:
    ros::NodeHandle nh_;
    std::vector<ros::Publisher> ugv_pubs_;  // UGV 퍼블리셔들
    std::vector<ros::Publisher> uav_pubs_;  // UAV 퍼블리셔들
    std::string frame_id_;        // 공통 frame_id
    std::string topic_name_;      // 공통 topic_name
    std::vector<Position> ugvs_{5};  // UGV 1~5 좌표
    std::vector<Position> uavs_{5};  // UAV 1~5 좌표

    void loadRobotPositions();
};

#endif // PUBLISH_INITIAL_POSE_HPP