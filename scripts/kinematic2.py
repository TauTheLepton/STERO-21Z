#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import tf

linear = 0.11
angular = 0.6
eps = 0.1
# global callback_data_x, callback_data_y, callback_data_orient
callback_data_x = 0.0
callback_data_y = 0.0
callback_data_orient = 0.0

def odom_callback(odometry):
   global callback_data_x, callback_data_y, callback_data_orient
   callback_data_x = odometry.pose.pose.position.x
   callback_data_y = odometry.pose.pose.position.y
   callback_data_orient = tf.transformations.euler_from_quaternion((0, 0, odometry.pose.pose.orientation.z, 0))[2]


def talker(side):
   pub = rospy.Publisher('/key_vel', Twist, queue_size=10)
   sub = rospy.Subscriber('/mobile_base_controller/odom', Odometry, odom_callback)
   rospy.init_node('kinematic', anonymous=True)
   rate = rospy.Rate(10) # 10hz 0.1s
   vel_msg = Twist()
   goal_pose_x = callback_data_x + side
   goal_pose_y = callback_data_y
   goal_orinet = callback_data_orient
   print(callback_data_orient)

   print("Let's move your ass!")
  
   while not rospy.is_shutdown():
      while math.sqrt((goal_pose_x - callback_data_x)**2 + (goal_pose_y - callback_data_y)**2) > eps:
         print(math.sqrt((goal_pose_x - callback_data_x)**2 + (goal_pose_y - callback_data_y)**2))
         vel_msg.linear.x = linear
         vel_msg.angular.z = 0
         pub.publish(vel_msg)
         rate.sleep()
      goal_orinet += math.pi/2
      while abs(abs(goal_orinet) - abs(callback_data_orient)) > eps:
         print(abs(abs(goal_orinet) - abs(callback_data_orient)))
         print(callback_data_orient)
         vel_msg.linear.x = 0
         vel_msg.angular.z = angular
         pub.publish(vel_msg)
         rate.sleep()
      # for i in range(iters_ang):
      #    vel_msg.linear.x = 0
      #    vel_msg.angular.z = angular
      #    pub.publish(vel_msg)
      #    rate.sleep()

      # count -= 1


if __name__ == '__main__':
   try:
      side = input("Podaj dlugosc boku: ")
      talker(side)
   except rospy.ROSInterruptException:
      pass