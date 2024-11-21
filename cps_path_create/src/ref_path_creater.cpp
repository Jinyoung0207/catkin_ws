#include "ref_path_creater.hpp"

double dis_calculator(double x1, double y1, double x2, double y2)
{
    double distance = std::abs(std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2)));
    return distance;
}

double pi_to_pi(double angle)
{
    while (angle >= M_PI)
        angle -= 2.0 * M_PI;

    while (angle < -M_PI)
        angle += 2.0 * M_PI;

    return angle;
}

double q_to_e(geometry_msgs::PoseStamped msg)
{
    tf::Quaternion q;
    q.setX(msg.pose.orientation.x);
    q.setY(msg.pose.orientation.y);
    q.setZ(msg.pose.orientation.z);
    q.setW(msg.pose.orientation.w);

    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    return yaw;
}

tf::Quaternion e_to_q(double roll, double pitch, double yaw)
{
    tf::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    // q.normalize();
    return q;
}

void ref_path_cps::global_path_Callback(const nav_msgs::Path::ConstPtr& msg)
{
    buf_.lock();
    global_path_queue_.push(*msg);
    buf_.unlock();
}

void ref_path_cps::set_init_formation()
{
    ld_x_ = 0.0;
    ld_y_ = 0.0;
    f1_x_ = 0.0;
    f1_y_ = -3.0;
    f2_x_ = 0.0;
    f2_y_ = 3.0;
    f3_x_ = 0.0 ;
    f3_y_ = -6.0 ;
    f4_x_ = 0.0;
    f4_y_ = 6.0;

    ld_x_uav = 0.0;
    ld_y_uav = 0.0;
    ld_z_uav = 2.0;
    f1_x_uav = 0.0;
    f1_y_uav = -3.0;
    f1_z_uav = 2.0;
    f2_x_uav = 0.0;
    f2_y_uav = 3.0;
    f2_z_uav = -3.0;
    f3_x_uav = 0.0 ;
    f3_y_uav = -6.0;
    f3_z_uav = 2.0;
    f4_x_uav = 0.0;
    f4_y_uav = 6.0;
    f4_z_uav = 2.0;
}

void ref_path_cps::ugv_formation_Callback(const geometry_msgs::PoseArray::ConstPtr& msg)
{
    if(!msg->poses.empty())
    {
        ld_x_ = msg->poses[0].position.x;
        ld_y_ = msg->poses[0].position.y;
        f1_x_ = msg->poses[1].position.x;
        f1_y_ = msg->poses[1].position.y;
        f2_x_ = msg->poses[2].position.x;
        f2_y_ = msg->poses[2].position.y;
        f3_x_ = msg->poses[3].position.x;
        f3_y_ = msg->poses[3].position.y;
        f4_x_ = msg->poses[4].position.x;
        f4_y_ = msg->poses[4].position.y;
    }
}

void ref_path_cps::uav_formation_Callback(const geometry_msgs::PoseArray::ConstPtr& msg)
{
    if(!msg->poses.empty())
    {
        ld_x_uav = msg->poses[0].position.x;
        ld_y_uav = msg->poses[0].position.y;
        ld_z_uav = msg->poses[0].position.z;
        f1_x_uav = msg->poses[1].position.x;
        f1_y_uav = msg->poses[1].position.y;
        f1_z_uav = msg->poses[1].position.z;
        f2_x_uav = msg->poses[2].position.x;
        f2_y_uav = msg->poses[2].position.y;
        f2_z_uav = msg->poses[2].position.z;
        f3_x_uav = msg->poses[3].position.x;
        f3_y_uav = msg->poses[3].position.y;
        f3_z_uav = msg->poses[3].position.z;
        f4_x_uav = msg->poses[4].position.x;
        f4_y_uav = msg->poses[4].position.y;
        f4_z_uav = msg->poses[4].position.z;
    }
}

void ref_path_cps::reset()
{
    lead_ugv_ref_traj_msg.poses.clear();
    f1_ugv_ref_traj_msg.poses.clear();
    f2_ugv_ref_traj_msg.poses.clear();
    f3_ugv_ref_traj_msg.poses.clear();
    f4_ugv_ref_traj_msg.poses.clear();

    lead_uav_ref_traj_msg.poses.clear();
    f1_uav_ref_traj_msg.poses.clear();
    f2_uav_ref_traj_msg.poses.clear();
    f3_uav_ref_traj_msg.poses.clear();
    f4_uav_ref_traj_msg.poses.clear();

    if (make_ref_id_ == formation_ref_path_msg.poses.size())
    {
        formation_ref_path_msg.poses.clear();
        make_ref_id_ = 0;
        global_path_in_ = false;
        create_ref_end_ = false;
        std::cout << "\033[33mref_trajectory end && Please publish new global_path ...\033[0m" << std::endl;
        std::cout << std::endl;
    }
    else
    {
        ++make_ref_id_;
    }
}

void ref_path_cps::publish()
{   //ugv
    lead_ugv_ref_traj_msg.header.frame_id = frame_id_;
    f1_ugv_ref_traj_msg.header.frame_id = frame_id_;
    f2_ugv_ref_traj_msg.header.frame_id = frame_id_;
    f3_ugv_ref_traj_msg.header.frame_id = frame_id_;
    f4_ugv_ref_traj_msg.header.frame_id = frame_id_;

    lead_ugv_ref_traj_msg.header.stamp = ros::Time::now();
    f1_ugv_ref_traj_msg.header.stamp = ros::Time::now();
    f2_ugv_ref_traj_msg.header.stamp = ros::Time::now();
    f3_ugv_ref_traj_msg.header.stamp = ros::Time::now();
    f4_ugv_ref_traj_msg.header.stamp = ros::Time::now();

    lead_ugv_ref_traj_pub.publish(lead_ugv_ref_traj_msg);
    f1_ugv_ref_traj_pub.publish(f1_ugv_ref_traj_msg);
    f2_ugv_ref_traj_pub.publish(f2_ugv_ref_traj_msg);
    f3_ugv_ref_traj_pub.publish(f3_ugv_ref_traj_msg);
    f4_ugv_ref_traj_pub.publish(f4_ugv_ref_traj_msg);
    //uav
    lead_uav_ref_traj_msg.header.frame_id = frame_id_;
    f1_uav_ref_traj_msg.header.frame_id = frame_id_;
    f2_uav_ref_traj_msg.header.frame_id = frame_id_;
    f3_uav_ref_traj_msg.header.frame_id = frame_id_;
    f4_uav_ref_traj_msg.header.frame_id = frame_id_;

    lead_uav_ref_traj_msg.header.stamp = ros::Time::now();
    f1_uav_ref_traj_msg.header.stamp = ros::Time::now();
    f2_uav_ref_traj_msg.header.stamp = ros::Time::now();
    f3_uav_ref_traj_msg.header.stamp = ros::Time::now();
    f4_uav_ref_traj_msg.header.stamp = ros::Time::now();

    lead_uav_ref_traj_pub.publish(lead_uav_ref_traj_msg);
    f1_uav_ref_traj_pub.publish(f1_uav_ref_traj_msg);
    f2_uav_ref_traj_pub.publish(f2_uav_ref_traj_msg);
    f3_uav_ref_traj_pub.publish(f3_uav_ref_traj_msg);
    f4_uav_ref_traj_pub.publish(f4_uav_ref_traj_msg);
    
    geometry_msgs::PoseStamped shift_uav;

    shift_uav = lead_uav_ref_traj_msg.poses[0];
    shift_uav.pose.position.x += 2*scale_nor;
    lead_uav_point_pub.publish(shift_uav);

    shift_uav = f1_uav_ref_traj_msg.poses[0];
    shift_uav.pose.position.x += 2*scale_nor;
    shift_uav.pose.position.y += 3*scale_nor;
    f1_uav_point_pub.publish(shift_uav);

    shift_uav = f2_uav_ref_traj_msg.poses[0];
    shift_uav.pose.position.x += 2*scale_nor;
    shift_uav.pose.position.y -= 3*scale_nor;
    f2_uav_point_pub.publish(shift_uav);

    shift_uav = f3_uav_ref_traj_msg.poses[0];
    shift_uav.pose.position.x += 2*scale_nor;
    shift_uav.pose.position.y += 6*scale_nor;
    f3_uav_point_pub.publish(shift_uav);

    shift_uav = f4_uav_ref_traj_msg.poses[0];
    shift_uav.pose.position.x += 2*scale_nor;
    shift_uav.pose.position.y -= 6*scale_nor;
    f4_uav_point_pub.publish(shift_uav);

}

void ref_path_cps::make_ref_trajectory()
{
    double fir_angle = 0.0;

    for(int ref_id = make_ref_id_; ref_id < make_ref_id_ + node; ++ref_id)
    {
        //ugv
        geometry_msgs::PoseStamped lead_node;
        geometry_msgs::PoseStamped f1_node;
        geometry_msgs::PoseStamped f2_node;
        geometry_msgs::PoseStamped f3_node;
        geometry_msgs::PoseStamped f4_node;
        //uav
        geometry_msgs::PoseStamped lead_uav_node;
        geometry_msgs::PoseStamped f1_uav_node;
        geometry_msgs::PoseStamped f2_uav_node;
        geometry_msgs::PoseStamped f3_uav_node;
        geometry_msgs::PoseStamped f4_uav_node;
        
        if(ref_id >= formation_ref_path_msg.poses.size())
        {
            lead_node = formation_ref_path_msg.poses[formation_ref_path_msg.poses.size() - 1];
            f1_node = formation_ref_path_msg.poses[formation_ref_path_msg.poses.size()-1];
            f2_node = formation_ref_path_msg.poses[formation_ref_path_msg.poses.size()-1];
            f3_node = formation_ref_path_msg.poses[formation_ref_path_msg.poses.size()-1];
            f4_node = formation_ref_path_msg.poses[formation_ref_path_msg.poses.size()-1];
            //uav
            lead_uav_node = formation_ref_path_msg.poses[formation_ref_path_msg.poses.size() - 1];
            f1_uav_node = formation_ref_path_msg.poses[formation_ref_path_msg.poses.size()-1];
            f2_uav_node = formation_ref_path_msg.poses[formation_ref_path_msg.poses.size()-1];
            f3_uav_node = formation_ref_path_msg.poses[formation_ref_path_msg.poses.size()-1];
            f4_uav_node = formation_ref_path_msg.poses[formation_ref_path_msg.poses.size()-1];

        }
        else
        {
            lead_node = formation_ref_path_msg.poses[ref_id];
            f1_node = formation_ref_path_msg.poses[ref_id];
            f2_node = formation_ref_path_msg.poses[ref_id];
            f3_node = formation_ref_path_msg.poses[ref_id];
            f4_node = formation_ref_path_msg.poses[ref_id];
            //uav
            lead_uav_node = formation_ref_path_msg.poses[ref_id];
            f1_uav_node = formation_ref_path_msg.poses[ref_id];
            f2_uav_node = formation_ref_path_msg.poses[ref_id];
            f3_uav_node = formation_ref_path_msg.poses[ref_id];
            f4_uav_node = formation_ref_path_msg.poses[ref_id];
        }
 
        if(ref_id == make_ref_id_)
        {
            fir_angle = pi_to_pi(q_to_e(lead_node));
            fir_angle = pi_to_pi(q_to_e(formation_ref_path_msg.poses[ref_id]));
        }
        //ugv
        lead_node.pose.position.x += (ld_x_ * cos(fir_angle)) - (ld_y_ * sin(fir_angle));
        lead_node.pose.position.y += (ld_x_ * sin(fir_angle)) + (ld_y_ * cos(fir_angle));
        f1_node.pose.position.x += (f1_x_ * cos(fir_angle)) - (f1_y_ * sin(fir_angle));
        f1_node.pose.position.y += (f1_x_ * sin(fir_angle)) + (f1_y_ * cos(fir_angle));
        f2_node.pose.position.x += (f2_x_ * cos(fir_angle)) - (f2_y_ * sin(fir_angle));
        f2_node.pose.position.y += (f2_x_ * sin(fir_angle)) + (f2_y_ * cos(fir_angle));
        f3_node.pose.position.x += (f3_x_ * cos(fir_angle)) - (f3_y_ * sin(fir_angle));
        f3_node.pose.position.y += (f3_x_ * sin(fir_angle)) + (f3_y_ * cos(fir_angle));
        f4_node.pose.position.x += (f4_x_ * cos(fir_angle)) - (f4_y_ * sin(fir_angle));
        f4_node.pose.position.y += (f4_x_ * sin(fir_angle)) + (f4_y_ * cos(fir_angle));
        //uav
        lead_uav_node.pose.position.x += (ld_x_uav * cos(fir_angle)) - (ld_y_uav * sin(fir_angle));
        lead_uav_node.pose.position.y += (ld_x_uav * sin(fir_angle)) + (ld_y_uav * cos(fir_angle));
        lead_uav_node.pose.position.z = ld_z_uav;
        f1_uav_node.pose.position.x += (f1_x_uav * cos(fir_angle)) - (f1_y_uav * sin(fir_angle));
        f1_uav_node.pose.position.y += (f1_x_uav * sin(fir_angle)) + (f1_y_uav * cos(fir_angle));
        f1_uav_node.pose.position.z = f1_z_uav;
        f2_uav_node.pose.position.x += (f2_x_uav * cos(fir_angle)) - (f2_y_uav * sin(fir_angle));
        f2_uav_node.pose.position.y += (f2_x_uav * sin(fir_angle)) + (f2_y_uav * cos(fir_angle));
        f2_uav_node.pose.position.z = f2_z_uav;
        f3_uav_node.pose.position.x += (f3_x_uav * cos(fir_angle)) - (f3_y_uav * sin(fir_angle));
        f3_uav_node.pose.position.y += (f3_x_uav * sin(fir_angle)) + (f3_y_uav * cos(fir_angle));
        f3_uav_node.pose.position.z = f3_z_uav;
        f4_uav_node.pose.position.x += (f4_x_uav * cos(fir_angle)) - (f4_y_uav * sin(fir_angle));
        f4_uav_node.pose.position.y += (f4_x_uav * sin(fir_angle)) + (f4_y_uav * cos(fir_angle));
        f4_uav_node.pose.position.z = f4_z_uav;
        //ugv
        lead_ugv_ref_traj_msg.poses.emplace_back(lead_node);
        f1_ugv_ref_traj_msg.poses.emplace_back(f1_node);
        f2_ugv_ref_traj_msg.poses.emplace_back(f2_node);
        f3_ugv_ref_traj_msg.poses.emplace_back(f3_node);
        f4_ugv_ref_traj_msg.poses.emplace_back(f4_node);
        //uav
        lead_uav_ref_traj_msg.poses.emplace_back(lead_uav_node);
        f1_uav_ref_traj_msg.poses.emplace_back(f1_uav_node);
        f2_uav_ref_traj_msg.poses.emplace_back(f2_uav_node);
        f3_uav_ref_traj_msg.poses.emplace_back(f3_uav_node);
        f4_uav_ref_traj_msg.poses.emplace_back(f4_uav_node);
    }
}

void ref_path_cps::make_formation_ref_trajectory()
{
    bool sampling_end = false;

    if(global_path_msg_.poses.size() > 1)
    {
        int sampling_node_num;
        double prev_node_x = global_path_msg_.poses[0].pose.position.x;
        double prev_node_y = global_path_msg_.poses[0].pose.position.y;
        

        for(int i = 0; i < global_path_msg_.poses.size(); ++i)
        {
            double cur_node_x = global_path_msg_.poses[i].pose.position.x;
            double cur_node_y = global_path_msg_.poses[i].pose.position.y;

            double node_distance = dis_calculator(cur_node_x, cur_node_y, prev_node_x, prev_node_y);

            sampling_node_num = (int)std::ceil(node_distance / (v_mean_ * (1/rate_)));

            for(int j = 0; j < sampling_node_num; ++j)
            {
                geometry_msgs::PoseStamped sampling_node;
                geometry_msgs::PoseStamped sampling_uav_node;
                sampling_node.pose.position.x = prev_node_x + ((cur_node_x - prev_node_x) / (double)sampling_node_num) * j;
                sampling_node.pose.position.y = prev_node_y + ((cur_node_y - prev_node_y) / (double)sampling_node_num) * j;

                sampling_uav_node.pose.position.x = prev_node_x + ((cur_node_x - prev_node_x) / (double)sampling_node_num) * j;
                sampling_uav_node.pose.position.y = prev_node_y + ((cur_node_y - prev_node_y) / (double)sampling_node_num) * j;
                sampling_uav_node.pose.position.z = 2.0;

                formation_ref_path_msg.poses.emplace_back(sampling_node);
                formation_uav_ref_path_msg.poses.emplace_back(sampling_uav_node);
            }

            prev_node_x = cur_node_x;
            prev_node_y = cur_node_y;

            if(i == global_path_msg_.poses.size() - 1)
            {
                formation_ref_path_msg.poses.emplace_back(global_path_msg_.poses[i]);
                formation_uav_ref_path_msg.poses.emplace_back(global_path_msg_.poses[i]);
                sampling_end = true;
            }
            // std::cout << formation_ref_path_msg.poses.size() << std::endl;
        }

        if(sampling_end)
        {
            double sam_prev_node_x = formation_ref_path_msg.poses[0].pose.position.x;
            double sam_prev_node_y = formation_ref_path_msg.poses[0].pose.position.y;

            for(int num = 1; num < formation_ref_path_msg.poses.size(); ++num)
            {
                double ref_angle = atan2(formation_ref_path_msg.poses[num].pose.position.y - sam_prev_node_y,
                                         formation_ref_path_msg.poses[num].pose.position.x - sam_prev_node_x);

                tf::Quaternion q = e_to_q(0.0, 0.0, pi_to_pi(ref_angle));
                last_head = q;

                formation_ref_path_msg.poses[formation_ref_path_msg.poses.size() - 1].pose.orientation = formation_ref_path_msg.poses[formation_ref_path_msg.poses.size() - 2].pose.orientation;
                formation_ref_path_msg.poses[num - 1].pose.orientation.x = q.x();
                formation_ref_path_msg.poses[num - 1].pose.orientation.y = q.y();
                formation_ref_path_msg.poses[num - 1].pose.orientation.z = q.z();
                formation_ref_path_msg.poses[num - 1].pose.orientation.w = q.w();

                sam_prev_node_x = formation_ref_path_msg.poses[num].pose.position.x;
                sam_prev_node_y = formation_ref_path_msg.poses[num].pose.position.y;

                last_head = q;
            }

            formation_ref_path_msg.poses.back().pose.orientation = formation_ref_path_msg.poses[formation_ref_path_msg.poses.size() - 2].pose.orientation;
            create_ref_end_ = true;
            // std::cout << GREEN"formation ref traj create end ... "END << std::endl;
        }
    }

    else
    {
        for(int i = 0; i < node; ++i)
        {
            geometry_msgs::PoseStamped sampling_node;
            sampling_node = global_path_msg_.poses[0];
            sampling_node.pose.orientation.x = last_head.x();
            sampling_node.pose.orientation.y = last_head.y();
            sampling_node.pose.orientation.z = last_head.z();
            sampling_node.pose.orientation.w = last_head.w();
            formation_ref_path_msg.poses.emplace_back(sampling_node);
        }
    }
}

void ref_path_cps::global_path_update()
{
    if(!global_path_queue_.empty())
    {
        std::lock_guard<std::mutex> lock(buf_);
        global_path_msg_ = global_path_queue_.front();
        global_path_queue_.pop();
        global_path_in_ = true;

        std::cout << GREEN"global path in ... "END << std::endl;
        
    } 
}

void ref_path_cps::read_param()
{
    ros::NodeHandle nh("~");  // Use private namespace for node handle

    // Read parameters with default values if not found
    nh.getParam("Hz", rate_);  
    nh.getParam("N", node);    
    nh.getParam("dt", dt_);   
    nh.getParam("v_mean", v_mean_); 
    nh.getParam("global_path_topic_name", global_path_topic_);  
    //ugv
    nh.getParam("lead_ugv_ref_topic_name", lead_ugv_ref_traj_topic);  
    nh.getParam("follow_1_ugv_ref_topic_name", f1_ugv_ref_traj_topic);  
    nh.getParam("follow_2_ugv_ref_topic_name", f2_ugv_ref_traj_topic);  
    nh.getParam("follow_3_ugv_ref_topic_name", f3_ugv_ref_traj_topic);  
    nh.getParam("follow_4_ugv_ref_topic_name", f4_ugv_ref_traj_topic);
    //uav
    nh.getParam("lead_uav_ref_topic_name", lead_uav_ref_traj_topic);  
    nh.getParam("follow_1_uav_ref_topic_name", f1_uav_ref_traj_topic);  
    nh.getParam("follow_2_uav_ref_topic_name", f2_uav_ref_traj_topic);  
    nh.getParam("follow_3_uav_ref_topic_name", f3_uav_ref_traj_topic);  
    nh.getParam("follow_4_uav_ref_topic_name", f4_uav_ref_traj_topic);  
    nh.getParam("frame_id", frame_id_);  // Default value "odom"

    // Print out the parameters for verification
    std::cout << "// ================================================ //" << std::endl;
    std::cout << "// Hz                      : " << rate_ << std::endl;
    std::cout << "// Node                    : " << node << std::endl;
    std::cout << "// dt                      : " << dt_ << std::endl;
    std::cout << "// v_mean                  : " << v_mean_ << std::endl;
    std::cout << "// global_path_topic_name  : " << global_path_topic_ << std::endl;
    std::cout << "// lead_ugv_ref_topic_name     : " << lead_ugv_ref_traj_topic << std::endl;
    std::cout << "// follow_1_ugv_ref_topic_name : " << f1_ugv_ref_traj_topic << std::endl;
    std::cout << "// follow_2_ugv_ref_topic_name : " << f2_ugv_ref_traj_topic << std::endl;
    std::cout << "// follow_3_ugv_ref_topic_name : " << f3_ugv_ref_traj_topic << std::endl;
    std::cout << "// follow_4_ugv_ref_topic_name : " << f4_ugv_ref_traj_topic << std::endl;
    std::cout << "// lead_uav_ref_topic_name     : " << lead_uav_ref_traj_topic << std::endl;
    std::cout << "// follow_1_uav_ref_topic_name : " << f1_uav_ref_traj_topic << std::endl;
    std::cout << "// follow_2_uav_ref_topic_name : " << f2_uav_ref_traj_topic << std::endl;
    std::cout << "// follow_3_uav_ref_topic_name : " << f3_uav_ref_traj_topic << std::endl;
    std::cout << "// follow_4_uav_ref_topic_name : " << f4_uav_ref_traj_topic << std::endl;
    std::cout << "// frame_id                : " << frame_id_ << std::endl;
    std::cout << "// ================================================ //" << std::endl;
}

void ref_path_cps::init()
{
    ros::NodeHandle nh;

    // formation_sub_ = nh.subscribe("/robot_formation", 1, &ref_path_cps::formation_Callback, this);
    global_path_sub_ = nh.subscribe("/cps_global_path", 1, &ref_path_cps::global_path_Callback, this);
    ugv_formation_sub = nh.subscribe("/ugv_formation_list", 1, &ref_path_cps::ugv_formation_Callback, this);
    uav_formation_sub = nh.subscribe("/uav_formation_list", 1, &ref_path_cps::uav_formation_Callback, this);


    //ugv
    lead_ugv_ref_traj_pub = nh.advertise<nav_msgs::Path>(lead_ugv_ref_traj_topic, 1);
    f1_ugv_ref_traj_pub = nh.advertise<nav_msgs::Path>(f1_ugv_ref_traj_topic, 1);
    f2_ugv_ref_traj_pub = nh.advertise<nav_msgs::Path>(f2_ugv_ref_traj_topic, 1);
    f3_ugv_ref_traj_pub = nh.advertise<nav_msgs::Path>(f3_ugv_ref_traj_topic, 1);
    f4_ugv_ref_traj_pub = nh.advertise<nav_msgs::Path>(f4_ugv_ref_traj_topic, 1);
    //uav
    lead_uav_ref_traj_pub = nh.advertise<nav_msgs::Path>(lead_uav_ref_traj_topic, 1);
    f1_uav_ref_traj_pub = nh.advertise<nav_msgs::Path>(f1_uav_ref_traj_topic, 1);
    f2_uav_ref_traj_pub = nh.advertise<nav_msgs::Path>(f2_uav_ref_traj_topic, 1);
    f3_uav_ref_traj_pub = nh.advertise<nav_msgs::Path>(f3_uav_ref_traj_topic, 1);
    f4_uav_ref_traj_pub = nh.advertise<nav_msgs::Path>(f4_uav_ref_traj_topic, 1);

    lead_uav_point_pub = nh.advertise<geometry_msgs::PoseStamped>("/uav0/mavros/setpoint_position/local", 1);
    f1_uav_point_pub = nh.advertise<geometry_msgs::PoseStamped>("/uav1/mavros/setpoint_position/local", 1);
    f2_uav_point_pub = nh.advertise<geometry_msgs::PoseStamped>("/uav2/mavros/setpoint_position/local", 1);
    f3_uav_point_pub = nh.advertise<geometry_msgs::PoseStamped>("/uav3/mavros/setpoint_position/local", 1);
    f4_uav_point_pub = nh.advertise<geometry_msgs::PoseStamped>("/uav4/mavros/setpoint_position/local", 1);
}

void ref_path_cps::run()
{
    // key_input_check();
    // if(key_board_in_)
    // {
    global_path_update();
    if (global_path_in_)
    {
        if (!create_ref_end_)
        {   
            make_formation_ref_trajectory();
        }
        make_ref_trajectory();
        publish();
        reset();
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "ref_path_creater");
    ros::NodeHandle nh;

    ref_path_cps ref_path_creater(nh);

    // Set the loop rate (adjust as needed)
    ros::Rate loop_rate(10);  // 10 Hz
    ref_path_creater.read_param();
    // Main loop
    ref_path_creater.init();
    ref_path_creater.set_init_formation();
    while (ros::ok()) {
        ref_path_creater.run();
        ros::spinOnce();  
        loop_rate.sleep();  
    }

    return 0;
}

