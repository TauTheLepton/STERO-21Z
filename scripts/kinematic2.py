#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
from tf.transformations import euler_from_quaternion

linear = 0.11
angular = -0.1
eps_lin = 0.1
eps_ang = 0.025

callback_data_x = 0.0
callback_data_y = 0.0
callback_data_orient = 0.0

# calculates the actual angle to travel taking into account the direction of rotation
def calc_angle_diff(actual, goal, speed):
   """
   params:
   actual: actual rotational position
   goal: desired rotational position
   speed: speed of rotation (sign indicates direction)
   """
   if speed > 0:
      if goal > actual:
         diff = goal - actual
      else:
         diff = math.pi * 2 - (actual - goal)
   else:
      if goal < actual:
         diff = actual - goal
      else:
         diff = math.pi * 2 - (goal - actual)
   return diff

# calculates distance in 2D
def calc_distance(actualX, actualY, goalX, goalY):
   return math.sqrt((goalX - actualX) ** 2 + (goalY - actualY) ** 2)

def odom_callback(odometry):
   global callback_data_x, callback_data_y, callback_data_orient
   callback_data_x = odometry.pose.pose.position.x
   callback_data_y = odometry.pose.pose.position.y
   orient = odometry.pose.pose.orientation
   callback_data_orient = euler_from_quaternion((orient.x, orient.y, orient.z, orient.w))[2]
   if callback_data_orient < 0:
      callback_data_orient = callback_data_orient + math.pi * 2

def talker(side):
   pub = rospy.Publisher('/key_vel', Twist, queue_size=10)
   sub = rospy.Subscriber('/mobile_base_controller/odom', Odometry, odom_callback)
   rospy.init_node('kinematic', anonymous=True)
   rate = rospy.Rate(10) # 10hz 0.1s
   vel_msg = Twist()
   goal_orient = callback_data_orient
   print("Let's move your ass!")
   count = 4
   while not rospy.is_shutdown() and count > 0:
      # setting goal position and orientation
      goal_pose_x = callback_data_x + side * math.cos(callback_data_orient)
      goal_pose_y = callback_data_y + side * math.sin(callback_data_orient)
      if angular > 0:
         goal_orient += math.pi / 2
      else:
         goal_orient -= math.pi / 2
      # normalizing orientation
      if goal_orient > math.pi * 2:
         goal_orient -= math.pi * 2
      elif goal_orient < 0:
         goal_orient += math.pi * 2
      # linear movement
      while calc_distance(callback_data_x, callback_data_y, goal_pose_x, goal_pose_y) > eps_lin:
         print("goal", goal_pose_x, goal_pose_y)
         print("actual", callback_data_x, callback_data_y)
         print("distance", calc_distance(callback_data_x, callback_data_y, goal_pose_x, goal_pose_y))
         vel_msg.linear.x = linear
         vel_msg.angular.z = 0
         pub.publish(vel_msg)
         rate.sleep()
      # rotation
      while calc_angle_diff(callback_data_orient, goal_orient, angular) > eps_ang:
         print("goal", goal_orient)
         print("actual", callback_data_orient)
         print("distance", calc_angle_diff(callback_data_orient, goal_orient, angular))
         vel_msg.linear.x = 0
         vel_msg.angular.z = angular
         pub.publish(vel_msg)
         rate.sleep()
      count -= 1

if __name__ == '__main__':
   try:
      side = input("Podaj dlugosc boku: ")
      talker(side)
   except rospy.ROSInterruptException:
      pass
