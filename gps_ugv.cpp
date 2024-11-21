#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <string>
#include <iostream>
#include <cmath>
#include <std_msgs/Int64.h>
#include <std_msgs/Float32.h>

#define   END    "\033[0m"
#define   RED    "\033[1;31m"
#define   GREEN  "\033[1;32m"
#define   BLUE   "\033[1;34m"
#define   CYAN   "\033[1;36m"
#define   YELL   "\033[1;33m"


sensor_msgs::NavSatFix UGV1_gps;
sensor_msgs::NavSatFix UGV2_gps;
sensor_msgs::NavSatFix UGV3_gps;
sensor_msgs::NavSatFix UGV4_gps;
sensor_msgs::NavSatFix UGV5_gps;



ros::Subscriber UGV1_gps_sub;
ros::Subscriber UGV2_gps_sub;
ros::Subscriber UGV3_gps_sub;
ros::Subscriber UGV4_gps_sub;
ros::Subscriber UGV5_gps_sub;

ros::Publisher ugv_initial_pose1_pub;
ros::Publisher ugv_initial_pose2_pub;
ros::Publisher ugv_initial_pose3_pub;
ros::Publisher ugv_initial_pose4_pub;
ros::Publisher ugv_initial_pose5_pub;


void UGV1_global_Callback(const sensor_msgs::NavSatFix::ConstPtr&msg)
{
    UGV1_gps=*msg;
}

void UGV2_global_Callback(const sensor_msgs::NavSatFix::ConstPtr&msg)
{
    UGV2_gps=*msg;
}

void UGV3_global_Callback(const sensor_msgs::NavSatFix::ConstPtr&msg)
{
    UGV3_gps=*msg;
}

void UGV4_global_Callback(const sensor_msgs::NavSatFix::ConstPtr&msg)
{
    UGV4_gps=*msg;
}

void UGV5_global_Callback(const sensor_msgs::NavSatFix::ConstPtr&msg)
{
    UGV5_gps=*msg;
}


// 지구 반지름 (미터 단위)
const double R = 6371000.0;

double UGV1_dis;
double UGV2_dis;
double UGV3_dis;
double UGV4_dis;
double UGV5_dis;


double safe_d;


geometry_msgs::PoseStamped UGV1_initial_pose;
geometry_msgs::PoseStamped UGV2_initial_pose;
geometry_msgs::PoseStamped UGV3_initial_pose;
geometry_msgs::PoseStamped UGV4_initial_pose;
geometry_msgs::PoseStamped UGV5_initial_pose;


void init(ros::NodeHandle &nh)
{
    UGV1_gps_sub = nh.subscribe("/UGV1/mavros/global_position/global", 1, &UGV1_global_Callback);
    UGV2_gps_sub = nh.subscribe("/UGV2/mavros/global_position/global", 1, &UGV2_global_Callback);
    UGV3_gps_sub = nh.subscribe("/UGV3/mavros/global_position/global", 1, &UGV3_global_Callback);
    UGV4_gps_sub = nh.subscribe("/UGV4/mavros/global_position/global", 1, &UGV4_global_Callback);
    UGV5_gps_sub = nh.subscribe("/UGV5/mavros/global_position/global", 1, &UGV5_global_Callback);

    ugv_initial_pose1_pub = nh.advertise<geometry_msgs::PoseStamped>("UGV1/initial_pose",1);
    ugv_initial_pose2_pub = nh.advertise<geometry_msgs::PoseStamped>("UGV2/initial_pose",1);
    ugv_initial_pose3_pub = nh.advertise<geometry_msgs::PoseStamped>("UGV3/initial_pose",1);
    ugv_initial_pose4_pub = nh.advertise<geometry_msgs::PoseStamped>("UGV4/initial_pose",1);
    ugv_initial_pose5_pub = nh.advertise<geometry_msgs::PoseStamped>("UGV5/initial_pose",1);
}


geometry_msgs::PoseStamped trans_xyz(sensor_msgs::NavSatFix UGVn_gps)
{
    double latRad = UGVn_gps.latitude * M_PI / 180.0;
    double lonRad = UGVn_gps.longitude * M_PI / 180.0;
    double x = (R +  UGVn_gps.altitude) * cos(latRad) * cos(lonRad);
    double y = (R +  UGVn_gps.altitude) * cos(latRad) * sin(lonRad);
    double z = (R +  UGVn_gps.altitude) * sin(latRad);

    geometry_msgs::PoseStamped trans_pose;
    trans_pose.pose.position.x=x;
    trans_pose.pose.position.y=y;
    trans_pose.pose.position.z=z;

    return trans_pose;
}

double dis_cal(geometry_msgs::PoseStamped a, geometry_msgs::PoseStamped b) 
{
    double dis= std::sqrt(std::pow((b.pose.position.x) - (a.pose.position.x), 2) + std::pow((b.pose.position.y) - (a.pose.position.y), 2) + std::pow((b.pose.position.z) - (a.pose.position.z), 2));
    
    return dis;
}

void init_publish()
{

/////////////////////uGV///////////////////////
    UGV1_initial_pose.pose.position.y= dis_cal(trans_xyz(UGV1_gps),trans_xyz(UGV1_gps));
    UGV2_initial_pose.pose.position.y= -dis_cal(trans_xyz(UGV1_gps),trans_xyz(UGV2_gps));
    UGV3_initial_pose.pose.position.y= dis_cal(trans_xyz(UGV1_gps),trans_xyz(UGV3_gps));
    UGV4_initial_pose.pose.position.y= -dis_cal(trans_xyz(UGV1_gps),trans_xyz(UGV4_gps));
    UGV5_initial_pose.pose.position.y= dis_cal(trans_xyz(UGV1_gps),trans_xyz(UGV5_gps));

    UGV1_initial_pose.pose.position.x=-2;
    UGV1_initial_pose.pose.position.z=0;

    UGV2_initial_pose.pose.position.x=-2;
    UGV2_initial_pose.pose.position.z=0;

    UGV3_initial_pose.pose.position.x=-2;
    UGV3_initial_pose.pose.position.z=0;

    UGV4_initial_pose.pose.position.x=-2;
    UGV4_initial_pose.pose.position.z=0;

    UGV5_initial_pose.pose.position.x=-2;
    UGV5_initial_pose.pose.position.z=0;

    ugv_initial_pose1_pub.publish(UGV1_initial_pose);
    ugv_initial_pose2_pub.publish(UGV2_initial_pose);
    ugv_initial_pose3_pub.publish(UGV3_initial_pose);
    ugv_initial_pose4_pub.publish(UGV4_initial_pose);
    ugv_initial_pose5_pub.publish(UGV5_initial_pose);
}
void run()
{
    double dis_12=dis_cal(trans_xyz(UGV1_gps),trans_xyz(UGV2_gps));
    double dis_13=dis_cal(trans_xyz(UGV1_gps),trans_xyz(UGV3_gps));
    double dis_14=dis_cal(trans_xyz(UGV1_gps),trans_xyz(UGV4_gps));
    double dis_15=dis_cal(trans_xyz(UGV1_gps),trans_xyz(UGV5_gps));

    double dis_23=dis_cal(trans_xyz(UGV2_gps),trans_xyz(UGV3_gps));
    double dis_24=dis_cal(trans_xyz(UGV2_gps),trans_xyz(UGV4_gps));
    double dis_25=dis_cal(trans_xyz(UGV2_gps),trans_xyz(UGV5_gps));

    double dis_34=dis_cal(trans_xyz(UGV3_gps),trans_xyz(UGV4_gps));
    double dis_35=dis_cal(trans_xyz(UGV3_gps),trans_xyz(UGV5_gps));

    double dis_45=dis_cal(trans_xyz(UGV4_gps),trans_xyz(UGV5_gps));

    std::cout << std::fixed;
    std::cout.precision(3);

    std::cout<<"safe_distance: "<<GREEN<<safe_d<<END <<std::endl;

    std::cout<<BLUE "UGV1 "END<<"&";
    if(std::sqrt(std::pow(dis_12,2))<safe_d)
    {
        std::cout<<RED" UGV2"<<": "<< std::sqrt(std::pow(dis_12,2))<< END;
    }
    else
    {
        std::cout<<" UGV2"<<": "<< std::sqrt(std::pow(dis_12,2));
    }
    if(std::sqrt(std::pow(dis_13,2))<safe_d)
    {
        std::cout<<RED" UGV3"<<": "<< std::sqrt(std::pow(dis_13,2))<< END;
    }
    else
    {
        std::cout<<" UGV3"<<": "<< std::sqrt(std::pow(dis_13,2));
    }
    if(std::sqrt(std::pow(dis_14,2))<safe_d)
    {
        std::cout<<RED" UGV4"<<": "<< std::sqrt(std::pow(dis_14,2))<< END;
    }
    else
    {
        std::cout<<" UGV4"<<": "<< std::sqrt(std::pow(dis_14,2));
    }
    if(std::sqrt(std::pow(dis_15,2))<safe_d)
    {
        std::cout<<RED" UGV5"<<": "<< std::sqrt(std::pow(dis_15,2))<< END<<std::endl;
    }
    else
    {
        std::cout<<" UGV5"<<": "<< std::sqrt(std::pow(dis_15,2))<<std::endl;
    }

    std::cout<<BLUE "UGV2 "END<<"&";
    if(std::sqrt(std::pow(dis_23,2))<safe_d)
    {
        std::cout<<RED" UGV3"<<": "<< std::sqrt(std::pow(dis_23,2))<< END;
    }
    else
    {
        std::cout<<" UGV3"<<": "<< std::sqrt(std::pow(dis_23,2));
    }
    if(std::sqrt(std::pow(dis_24,2))<safe_d)
    {
        std::cout<<RED" UGV4"<<": "<< std::sqrt(std::pow(dis_24,2))<< END;
    }
    else
    {
        std::cout<<" UGV4"<<": "<< std::sqrt(std::pow(dis_24,2));
    }
    if(std::sqrt(std::pow(dis_25,2))<safe_d)
    {
        std::cout<<RED" UGV5"<<": "<< std::sqrt(std::pow(dis_25,2))<< END<<std::endl;
    }
    else
    {
        std::cout<<" UGV5"<<": "<< std::sqrt(std::pow(dis_25,2))<<std::endl;
    }

    std::cout<<BLUE "UGV3 "END<<"&";
    if(std::sqrt(std::pow(dis_34,2))<safe_d)
    {
        std::cout<<RED" UGV4"<<": "<< std::sqrt(std::pow(dis_34,2))<< END;
    }
    else
    {
        std::cout<<" UGV4"<<": "<< std::sqrt(std::pow(dis_34,2));
    }
    if(std::sqrt(std::pow(dis_35,2))<safe_d)
    {
        std::cout<<RED" UGV5"<<": "<< std::sqrt(std::pow(dis_35,2))<< END<<std::endl;
    }
    else
    {
        std::cout<<" UGV5"<<": "<< std::sqrt(std::pow(dis_35,2))<<std::endl;
    }

    std::cout<<BLUE "UGV4 "END<<"&";
    if(std::sqrt(std::pow(dis_45,2))<safe_d)
    {
        std::cout<<RED" UGV5"<<": "<< std::sqrt(std::pow(dis_45,2))<< END<<std::endl;
    }
    else
    {
        std::cout<<" UGV5"<<": "<< std::sqrt(std::pow(dis_45,2))<<std::endl;
    }

    std::cout<<"-----------------------------------------------------------"<< std::endl;
    
}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "gps");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);  // 10 Hz
    // Main loop
    init(nh);
    param_read(nh);

    int i=0;

    while (ros::ok()) 
    {
        if(i<100)
        {
            init_publish();
            std::cout<<"publishing initial pose for (" << RED<<double(i)/10<< END <<"/10) sec..."<<std::endl;
            i=i+1;
        }
        else if(i==100)
        {
            run();
            i=i;
        }
        ros::spinOnce();  
        loop_rate.sleep();  
    }

    return 0;
}

