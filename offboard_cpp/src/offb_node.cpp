#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <cmath>

mavros_msgs::State current_state;

// 현재 상태 콜백 함수
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

int main(int argc, char **argv) {
    // 노드 초기화
    ros::init(argc, argv, "circle_node");
    ros::NodeHandle nh;

    // 상태 구독자 및 서비스 클라이언트 설정
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>(
        "mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>(
        "mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>(
        "mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>(
        "mavros/set_mode");

    // setpoint 전송 주기 설정
    ros::Rate rate(20.0);

    // FCU 연결 대기
    while (ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }

    // 원의 반지름 정의
    double radius = 2.0;

    // 원 운동을 위한 각속도 정의
    double angular_velocity = 0.5; // rad/s

    // 초기 각도 정의
    double angle = 0.0;

    // 원의 중심 설정
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    ros::Time last_request = ros::Time::now();

    while (ros::ok()) {
        // OFFBOARD 모드 설정 및 ARM 요청
        if (current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))) {
            if (set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent) {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if (!current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))) {
                if (arming_client.call(arm_cmd) &&
                    arm_cmd.response.success) {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        // 원 경로 상의 새로운 위치 계산
        pose.pose.position.x = radius * cos(angle);
        pose.pose.position.y = radius * sin(angle);

        // 새로운 위치 전송
        local_pos_pub.publish(pose);

        // 다음 반복을 위한 각도 업데이트
        angle += angular_velocity / 20.0; // 20Hz 주기를 고려하여

        // 각도가 [0, 2*pi) 범위 내에 있도록 보장
        if (angle >= 2 * M_PI) {
            angle -= 2 * M_PI;
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

