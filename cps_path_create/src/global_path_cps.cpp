#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <mutex>
#include <vector>

class GlobalPathCreate : public ros::NodeHandle
{
public:
    GlobalPathCreate()
    {
        // Publisher for global path
        global_path_pub_ = this->advertise<nav_msgs::Path>("/cps_global_path", 1);

        // Subscription to goal pose
        goal_pose_sub_ = this->subscribe("/move_base_simple/goal", 1, &GlobalPathCreate::goalCallback, this);
    }

    void run()
    {
        ros::Rate r(10); // 10Hz rate
        while (ros::ok())
        {
            ros::spinOnce();
            makePath();
            r.sleep();
        }
    }

private:
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        g_pose_ = {msg->pose.position.x, msg->pose.position.y};
        ROS_INFO("New goal received: [%.2f, %.2f]", g_pose_[0], g_pose_[1]);
    }

    void makePath()
    {
        std::lock_guard<std::mutex> lock(mutex_);

        if (g_pose_.empty())
        {
            ROS_WARN("Goal pose is empty, skipping path generation.");
            return;
        }

        nav_msgs::Path path;
        path.header.stamp = ros::Time::now();
        path.header.frame_id = "map";

        geometry_msgs::PoseStamped pose;
        pose.header = path.header;

        // Start point (0,0)
        pose.pose.position.x = 0.0;
        pose.pose.position.y = 0.0;
        path.poses.emplace_back(pose);

        // Goal point (g_pose_[0], g_pose_[1])
        pose.pose.position.x = g_pose_[0];
        pose.pose.position.y = g_pose_[1];
        path.poses.emplace_back(pose);

        global_path_pub_.publish(path);
    }

    ros::Publisher global_path_pub_;
    ros::Subscriber goal_pose_sub_;
    std::mutex mutex_;
    std::vector<double> g_pose_;
};

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "global_path_creator");
    GlobalPathCreate global_path_creator;

    global_path_creator.run();

    return 0;
}
