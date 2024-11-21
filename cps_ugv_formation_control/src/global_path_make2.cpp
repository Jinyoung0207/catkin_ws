#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include <cmath>
#include <vector>

class global_path_make2 {
public:
    global_path_make2() {
        ros::NodeHandle nh;
        publisher_ = nh.advertise<nav_msgs::Path>("/global_path2", 10);

        // Generate x values from 0 to 15 with step 0.5
        for (double x = 0.0; x <= 15.0; x += 0.5) {
            x_values.push_back(x);
            y_values.push_back(std::sin(x));  // y = 0 for all x
        }

        timer_ = nh.createTimer(ros::Duration(0.1), &global_path_make2::publishPath, this);
    }

private:
    void publishPath(const ros::TimerEvent&) {
        nav_msgs::Path path_msg;

        // Set header for the path message
        path_msg.header.stamp = ros::Time::now();
        path_msg.header.frame_id = "world";

        // Fill the path message with pose stamped messages
        for (size_t i = 0; i < x_values.size(); ++i) {
            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.header.stamp = path_msg.header.stamp;
            pose_stamped.header.frame_id = path_msg.header.frame_id;
            pose_stamped.pose.position.x = x_values[i];
            pose_stamped.pose.position.y = y_values[i];
            pose_stamped.pose.position.z = 0.0;  // Assuming z is zero
            pose_stamped.pose.orientation.w = 1.0;  // Quaternion identity

            path_msg.poses.push_back(pose_stamped);
        }

        publisher_.publish(path_msg);
    }

    std::vector<double> x_values;
    std::vector<double> y_values;
    ros::Publisher publisher_;
    ros::Timer timer_;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "global_path_make2");
    global_path_make2 path_publisher;
    ros::spin();
    return 0;
}