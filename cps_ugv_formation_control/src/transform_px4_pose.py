#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import queue
import numpy as np

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
RED = '\033[31m'
GREEN = '\033[32m'
BLUE = '\033[34m'
END = '\033[0m'

class TransformPose:
  def __init__(self):
    p_robot_name = rospy.search_param('robot')
    self.robot_name = '/'
    self.robot_name += rospy.get_param(p_robot_name)
    print(self.robot_name)
    self.local_pose_mean = np.zeros(7)
    self.pose_sub  = rospy.Subscriber(self.robot_name + '/mavros/local_position/pose', PoseStamped, self.odomCallback, queue_size = 1) # Local position from mavros
    self.mean_pose_pub = rospy.Publisher(self.robot_name + '/mean_pose', PoseStamped, queue_size=1)
    self.pose_pub = rospy.Publisher( self.robot_name + '/pose', PoseStamped, queue_size = 1)
    self.px4_pose = [0, 0, 0, 0, 0, 0, 0]
    self.pose_position = [0, 0, 0]
    self.pose_ori = [0, 0, 0, 0]
    self.robot_state_queue = queue.Queue()
    self.local_pose_queue = queue.Queue()

  def odomCallback(self, msg):
      self.local_pose_queue.put(item=msg)
      self.px4_pose = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z\
                              ,msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]

  def averagePoses(self):
    local_pose_history = []
    mean_x = 0.0; mean_y = 0.0; mean_z = 0.0; mean_ori_x = 0.0; mean_ori_y = 0.0; mean_ori_z = 0.0; mean_ori_w = 0.0
    while 1:
      local_pose_history.append(self.px4_pose)
      if len(local_pose_history) < 100:
        print('Averaging poses')
        mean_x += self.px4_pose[0]/100
        mean_y += self.px4_pose[1]/100
        mean_z += self.px4_pose[2]/100
        mean_ori_x += self.px4_pose[3]/100
        mean_ori_y += self.px4_pose[4]/100
        mean_ori_z += self.px4_pose[5]/100
        mean_ori_w += self.px4_pose[6]/100
  
      else:
        self.local_pose_mean[0] = mean_x
        self.local_pose_mean[1] = mean_y
        self.local_pose_mean[2] = mean_z
        self.local_pose_mean[3] = mean_ori_x
        self.local_pose_mean[4] = mean_ori_y
        self.local_pose_mean[5] = mean_ori_z
        self.local_pose_mean[6] = mean_ori_w
        print(GREEN+'Averaging_poses is completed!'+END)
        print('---------',self.robot_name,'Initial Pose----------')
        print('Position X : ', self.local_pose_mean[0])
        print('Position Y : ', self.local_pose_mean[1])
        print('Position Z : ', self.local_pose_mean[2])
        print('Orientation X : ', self.local_pose_mean[3])
        print('Orientation Y : ', self.local_pose_mean[4])
        print('Orientation Z : ', self.local_pose_mean[5])
        print('Orientation Z : ', self.local_pose_mean[6])
        break
    
  def publishMeanPos(self):
    mean_pose = PoseStamped()
    mean_pose.header.frame_id = 'map'
    mean_pose.header.stamp = rospy.Time.now()
    mean_pose.pose.position.x = self.local_pose_mean[0]
    mean_pose.pose.position.y = self.local_pose_mean[1]
    mean_pose.pose.position.z = self.local_pose_mean[2]
    mean_pose.pose.orientation.x = self.local_pose_mean[3]
    mean_pose.pose.orientation.y = self.local_pose_mean[4]
    mean_pose.pose.orientation.z = self.local_pose_mean[5]
    mean_pose.pose.orientation.w = self.local_pose_mean[6]
    self.mean_pose_pub.publish(mean_pose)

  def publishPose(self):
    pub_msg = PoseStamped()
    pub_msg.header.frame_id = 'map'
    pub_msg.header.stamp = rospy.Time.now()
    pub_msg.pose.position.x = self.px4_pose[0] - self.local_pose_mean[0]
    pub_msg.pose.position.y = self.px4_pose[1] - self.local_pose_mean[1]
    pub_msg.pose.position.z = self.px4_pose[2] - self.local_pose_mean[2]
    pub_msg.pose.orientation.x = self.px4_pose[3]
    pub_msg.pose.orientation.y = self.px4_pose[4]
    pub_msg.pose.orientation.z = self.px4_pose[5]
    pub_msg.pose.orientation.w = self.px4_pose[6]
    self.pose_pub.publish(pub_msg)
    
def main():
  rospy.init_node('transform_pose', anonymous=True)
  POSE = TransformPose()
  rate = rospy.Rate(10)
  while 1:
    if POSE.local_pose_queue.empty():
      print('local pose is not subscribed..')
    else: break
        
    print('\n')
    print('*' * 45)
    input(GREEN + 'Enter any number to start' + END + RED + ' Averaging Initial Pose \n' + END)
    
    POSE.averagePoses()
    POSE.publishMeanPos()
    while(not rospy.is_shutdown()):
      POSE.publishPose()
      rate.sleep()

if __name__ == "__main__":
  main()