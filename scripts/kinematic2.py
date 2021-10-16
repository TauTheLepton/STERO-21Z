#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import tf

linear = 0.11
angular = 0.6
eps = 0.1

callback_data_x = 0.0
callback_data_y = 0.0
callback_data_orient = 0.0

def odom_callback(odometry):
   global callback_data_x, callback_data_y, callback_data_orient
   callback_data_x = odometry.pose.pose.position.x
   callback_data_y = odometry.pose.pose.position.y
   callback_data_orient = tf.transformations.euler_from_quaternion((0, 0, odometry.pose.pose.orientation.z, odometry.pose.pose.orientation.w))[2]


def talker(side):
   pub = rospy.Publisher('/key_vel', Twist, queue_size=10)
   sub = rospy.Subscriber('/mobile_base_controller/odom', Odometry, odom_callback)
   rospy.init_node('kinematic', anonymous=True)
   rate = rospy.Rate(10) # 10hz 0.1s
   vel_msg = Twist()
   goal_pose_x = callback_data_x + side * math.cos(callback_data_orient)
   goal_pose_y = callback_data_y + side * math.sin(callback_data_orient)
   goal_orient = callback_data_orient + math.pi/2

   print("Let's move your ass!")
   count = 4
  
   while not rospy.is_shutdown() and count > 0:
      while math.sqrt((goal_pose_x - callback_data_x)**2 + (goal_pose_y - callback_data_y)**2) > eps:
         print("goal", goal_pose_x, goal_pose_y)
         print("actual", callback_data_x, callback_data_y)
         vel_msg.linear.x = linear
         vel_msg.angular.z = 0
         pub.publish(vel_msg)
         rate.sleep()
      while (goal_orient - callback_data_orient) > eps:
         print(goal_orient - callback_data_orient)
         vel_msg.linear.x = 0
         vel_msg.angular.z = angular
         pub.publish(vel_msg)
         rate.sleep()
      goal_pose_x = callback_data_x + side * math.cos(callback_data_orient)
      goal_pose_y = callback_data_y + side * math.sin(callback_data_orient)
      goal_orient += math.pi/2
      count -= 1


if __name__ == '__main__':
   try:
      side = input("Podaj dlugosc boku: ")
      talker(side)
   except rospy.ROSInterruptException:
      pass
