#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import math

linear = 0.11
angular = 0.1

def talker(side):
   pub = rospy.Publisher('/key_vel', Twist, queue_size=10)
   rospy.init_node('kinematic', anonymous=True)
   rate = rospy.Rate(10) # 10hz 0.1s
   vel_msg = Twist()
   print("Let's move your ass!")
   time_lin = side/linear
   time_ang = (math.pi/2)/angular
   iters_lin = int(time_lin / 0.1)
   iters_ang = int(time_ang / 0.1)
  
   count = 4
   while not rospy.is_shutdown() and count > 0:
      for i in range(iters_lin):
         vel_msg.linear.x = linear
         vel_msg.angular.z = 0
         pub.publish(vel_msg)
         rate.sleep()
      for i in range(iters_ang):
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
