#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <cmath>
#include <nav_msgs/Path.h> //path
#include <geometry_msgs/PointStamped.h> //p
#include <std_msgs/Int64.h>
#include <experimental/filesystem>



class Sendtrajectory 
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber pose_0_sub; // 각 uav0,1,2에 대한 pose 구독
    ros::Subscriber pose_1_sub;
    ros::Subscriber pose_2_sub; 
    ros::Subscriber pose_3_sub;
    ros::Subscriber num_sub;
    ros::Subscriber cmd_sub;
          
          

    ros::Publisher r_path_0_pub; //uav 0,1,2에 대한 reference path 발행
    ros::Publisher r_path_1_pub; 
    ros::Publisher r_path_2_pub; //ref_path_publish
    ros::Publisher r_path_3_pub;

    ros::Publisher path_0_pub; 
    ros::Publisher path_1_pub; 
    ros::Publisher path_2_pub; //current_path_publish
    ros::Publisher path_3_pub;

    ros::Publisher gpose_1_pub; 
    ros::Publisher gpose_2_pub; //uav1,2 global_pose_publish 
    ros::Publisher gpose_3_pub;

    geometry_msgs::PoseStamped current_pose_0;
    geometry_msgs::PoseStamped current_pose_1;
    geometry_msgs::PoseStamped current_pose_2;
    geometry_msgs::PoseStamped current_pose_3;

    nav_msgs::Path uav0_path; 
    nav_msgs::Path uav1_path; 
    nav_msgs::Path uav2_path;
    nav_msgs::Path uav3_path; 

    std_msgs::Int64 formation_num;

    

    void pose0Callback(const geometry_msgs::PoseStamped::ConstPtr& msg) 
    {
        current_pose_0 = *msg;
    }
    void pose1Callback(const geometry_msgs::PoseStamped::ConstPtr& msg) 
    {
        current_pose_1= *msg;
    }
    void pose2Callback(const geometry_msgs::PoseStamped::ConstPtr& msg) 
    {
        current_pose_2 = *msg;
    }
    void pose3Callback(const geometry_msgs::PoseStamped::ConstPtr& msg) 
    {
        current_pose_3 = *msg;
    }
    void numCallback(const std_msgs::Int64::ConstPtr& msg) 
    {
        formation_num = *msg;
    }
public:
    Sendtrajectory() : nh_(""), current_pose_0(),current_pose_1(),current_pose_2()
    {
        pose_0_sub = nh_.subscribe<geometry_msgs::PoseStamped>("uav0/mavros/local_position/pose", 10, &Sendtrajectory::pose0Callback, this);
        pose_1_sub = nh_.subscribe<geometry_msgs::PoseStamped>("uav1/mavros/local_position/pose", 10, &Sendtrajectory::pose1Callback, this);
        pose_2_sub = nh_.subscribe<geometry_msgs::PoseStamped>("uav2/mavros/local_position/pose", 10, &Sendtrajectory::pose2Callback, this);
        pose_3_sub = nh_.subscribe<geometry_msgs::PoseStamped>("uav3/mavros/local_position/pose", 10, &Sendtrajectory::pose3Callback, this);
        num_sub = nh_.subscribe<std_msgs::Int64>("num/formation/info", 10, &Sendtrajectory::numCallback, this);


        r_path_0_pub= nh_.advertise<nav_msgs::Path>("uav0/ref_path", 10);
        r_path_1_pub= nh_.advertise<nav_msgs::Path>("uav1/ref_path", 10);
        r_path_2_pub= nh_.advertise<nav_msgs::Path>("uav2/ref_path", 10);
        r_path_3_pub= nh_.advertise<nav_msgs::Path>("uav3/ref_path", 10);

        path_0_pub = nh_.advertise<nav_msgs::Path>("uav0/path", 10); 
        path_1_pub = nh_.advertise<nav_msgs::Path>("uav1/path", 10); 
        path_2_pub = nh_.advertise<nav_msgs::Path>("uav2/path", 10);
        path_3_pub = nh_.advertise<nav_msgs::Path>("uav3/path", 10); 

        gpose_1_pub = nh_.advertise<geometry_msgs::PoseStamped>("uav1/global_current_pose", 10);
        gpose_2_pub = nh_.advertise<geometry_msgs::PoseStamped>("uav2/global_current_pose", 10);
        gpose_3_pub = nh_.advertise<geometry_msgs::PoseStamped>("uav3/global_current_pose", 10);
    }

    void run() 
    {
        ros::Rate rate(5.0);

        nav_msgs::Path ref_path_0_msg;
        nav_msgs::Path ref_path_1_msg;
        nav_msgs::Path ref_path_2_msg;
        nav_msgs::Path ref_path_3_msg;

        geometry_msgs::PoseStamped refpoint0;
        geometry_msgs::PoseStamped refpoint1;
        geometry_msgs::PoseStamped refpoint2;
        geometry_msgs::PoseStamped refpoint3;


        geometry_msgs::PoseStamped gpose1;
        geometry_msgs::PoseStamped gpose2;
        geometry_msgs::PoseStamped gpose3;

        
        double temp_x,temp_y,x_r,y_r=0; 

        ref_path_0_msg.header.frame_id = "map";
        ref_path_1_msg.header.frame_id = "map";
        ref_path_2_msg.header.frame_id = "map";
        ref_path_3_msg.header.frame_id = "map";


        uav0_path.header.frame_id="map";
        uav1_path.header.frame_id="map";
        uav2_path.header.frame_id="map";
        uav3_path.header.frame_id="map";


        gpose1.header.frame_id="map";
        gpose2.header.frame_id="map";
        gpose3.header.frame_id="map";


        gpose1.pose.position.x=current_pose_1.pose.position.x-3; //좌표계 local to global
        gpose1.pose.position.y=current_pose_1.pose.position.y+3;
        gpose1.pose.position.z=current_pose_1.pose.position.z;

        gpose2.pose.position.x=current_pose_2.pose.position.x-3;
        gpose2.pose.position.y=current_pose_2.pose.position.y-3;
        gpose2.pose.position.z=current_pose_2.pose.position.z;

        gpose3.pose.position.x=current_pose_3.pose.position.x-6;
        gpose3.pose.position.y=current_pose_3.pose.position.y;
        gpose3.pose.position.z=current_pose_3.pose.position.z;


        double formation[2][4][3] = {{{0,0,0},{-3,3,0},{-3,-3,0},{-6,0,0}}, //포메이션별 refpoint에 더해줘야 할 값
                                    {{0,0,0},{-3,0,0},{-6,0,0},{-9,0,0}}};
        
        while (ros::ok()) 
        {
            if (current_pose_0.pose.position.z >= 2.0) 
            {
                for(int i = 0; i < 10; ++i) //예측 path이기 때문에 i를 1 더한 후 시작
                {
                    if (x_r < 35) 
                    {
                        x_r = temp_x + 0.1 * i; //x_r이 조금씩 커지는거지 //refpoint의 첫번째 값을 계속 더해주는거임
                        y_r = 0;
                        if(i == 1)
                        {   
                            temp_x = x_r;  //ref_path의 첫번째 예상 값을 temp에 저장
                            temp_y = y_r;  //10개의 값중 첫번째 값만 계속 저장하는거
                        }
                        if(formation_num.data == 1)
                        {
                            ROS_INFO("formation_num.data == 0");
                            refpoint0.pose.position.x=x_r + formation[0][0][0];
                            refpoint0.pose.position.y=y_r + formation[0][0][0];
                            refpoint0.pose.position.z=2 + formation[0][0][0];

                            refpoint1.pose.position.x=x_r -3 + formation[0][1][0];  
                            refpoint1.pose.position.y=y_r +3 + formation[0][1][1];  
                            refpoint1.pose.position.z=2 + formation[0][1][2];

                            refpoint2.pose.position.x=x_r -3 + formation[0][2][0];
                            refpoint2.pose.position.y=y_r -3 + formation[0][2][1];
                            refpoint2.pose.position.z=2 + formation[0][2][2];

                            refpoint3.pose.position.x=x_r -6 + formation[0][3][0];
                            refpoint3.pose.position.y=y_r + formation[0][3][1];
                            refpoint3.pose.position.z=2 + formation[0][3][2];

                            ref_path_0_msg.header.stamp = ros::Time::now();
                            ref_path_0_msg.poses.emplace_back(refpoint0); //값을 넣어주는 함수

                            ref_path_1_msg.header.stamp = ros::Time::now();
                            ref_path_1_msg.poses.emplace_back(refpoint1);

                            ref_path_2_msg.header.stamp = ros::Time::now();
                            ref_path_2_msg.poses.emplace_back(refpoint2);

                            ref_path_3_msg.header.stamp = ros::Time::now();
                            ref_path_3_msg.poses.emplace_back(refpoint3);
                        
                        }
                        else if (formation_num.data == 2)
                        {
                            ROS_INFO("formation_num.data == 2");
                            refpoint0.pose.position.x=x_r + formation[1][0][0];
                            refpoint0.pose.position.y=y_r + formation[1][0][0];
                            refpoint0.pose.position.z=2 + formation[1][0][0];

                            refpoint1.pose.position.x=x_r + formation[1][1][0];  
                            refpoint1.pose.position.y=y_r + formation[1][1][1];  
                            refpoint1.pose.position.z=2 + formation[1][1][2];

                            refpoint2.pose.position.x=x_r + formation[1][2][0];
                            refpoint2.pose.position.y=y_r + formation[1][2][1];
                            refpoint2.pose.position.z=2 + formation[1][2][2];

                            refpoint3.pose.position.x=x_r + formation[1][3][0];
                            refpoint3.pose.position.y=y_r + formation[1][3][1];
                            refpoint3.pose.position.z=2 + formation[1][3][2];

                            ref_path_0_msg.header.stamp = ros::Time::now();
                            ref_path_0_msg.poses.emplace_back(refpoint0); //값을 넣어주는 함수

                            ref_path_1_msg.header.stamp = ros::Time::now();
                            ref_path_1_msg.poses.emplace_back(refpoint1);

                            ref_path_2_msg.header.stamp = ros::Time::now();
                            ref_path_2_msg.poses.emplace_back(refpoint2);

                            ref_path_3_msg.header.stamp = ros::Time::now();
                            ref_path_3_msg.poses.emplace_back(refpoint3);
                        }
                    }             
                }
            }
            else
            {
                refpoint0.pose.position.x=x_r;
                refpoint0.pose.position.y=y_r;
                refpoint0.pose.position.z=2.0;

                refpoint1.pose.position.x=x_r-3;
                refpoint1.pose.position.y=y_r+3;
                refpoint1.pose.position.z=2.0;

                refpoint2.pose.position.x=x_r-3;
                refpoint2.pose.position.y=y_r-3;
                refpoint2.pose.position.z=2.0;

                refpoint3.pose.position.x=x_r-6;
                refpoint3.pose.position.y=y_r;
                refpoint3.pose.position.z=2.0;

                ref_path_0_msg.header.stamp = ros::Time::now();
                ref_path_0_msg.poses.emplace_back(refpoint0);

                ref_path_1_msg.header.stamp = ros::Time::now();
                ref_path_1_msg.poses.emplace_back(refpoint1);

                ref_path_2_msg.header.stamp = ros::Time::now();
                ref_path_2_msg.poses.emplace_back(refpoint2);

                ref_path_3_msg.header.stamp = ros::Time::now();
                ref_path_3_msg.poses.emplace_back(refpoint3);
            }

            gpose1.pose.position.x=current_pose_1.pose.position.x-3; //좌표계 local to global
            gpose1.pose.position.y=current_pose_1.pose.position.y+3;
            gpose1.pose.position.z=current_pose_1.pose.position.z;

            gpose2.pose.position.x=current_pose_2.pose.position.x-3;
            gpose2.pose.position.y=current_pose_2.pose.position.y-3;
            gpose2.pose.position.z=current_pose_2.pose.position.z;

            gpose3.pose.position.x=current_pose_3.pose.position.x-6;
            gpose3.pose.position.y=current_pose_3.pose.position.y;
            gpose3.pose.position.z=current_pose_3.pose.position.z;           
            
            //global_pose_publish       
            
            gpose1.header.stamp=ros::Time::now();
            gpose_1_pub.publish(gpose1);
            gpose2.header.stamp=ros::Time::now();
            gpose_2_pub.publish(gpose2);
            gpose3.header.stamp=ros::Time::now();
            gpose_3_pub.publish(gpose3);

             //path_publish
            uav0_path.header.stamp=ros::Time::now();
            uav0_path.poses.emplace_back(current_pose_0);
            path_0_pub.publish(uav0_path); 

            uav1_path.header.stamp=ros::Time::now();
            uav1_path.poses.emplace_back(gpose1); //local 좌표계에서 계산되어진 글로벌 좌표계 값이 들어가야됨
            path_1_pub.publish(uav1_path); 

            uav2_path.header.stamp=ros::Time::now();
            uav2_path.poses.emplace_back(gpose2);
            path_2_pub.publish(uav2_path);

            uav3_path.header.stamp=ros::Time::now();
            uav3_path.poses.emplace_back(gpose3);
            path_3_pub.publish(uav3_path);       

            //ref_path_publish
            r_path_0_pub.publish(ref_path_0_msg); // refpoint의 값이 들어감.
            ref_path_0_msg.poses.clear(); //안의 값을 비워주기 위해 초기화 시켜줌

            r_path_1_pub.publish(ref_path_1_msg);
            ref_path_1_msg.poses.clear();

            r_path_2_pub.publish(ref_path_2_msg);
            ref_path_2_msg.poses.clear();  

            r_path_3_pub.publish(ref_path_3_msg);
            ref_path_3_msg.poses.clear();              

            ros::spinOnce();
            rate.sleep();
        }
    }
};

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "velocity_traj");

    Sendtrajectory send_traj;
    send_traj.run();

    return 0;
}


