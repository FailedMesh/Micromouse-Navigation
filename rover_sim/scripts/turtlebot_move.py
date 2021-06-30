#! /usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist


def callback(data): 
  rospy.loginfo(" Battery remaining is -  %s",data.data)
  if data.data > 55:
      move.linear.x = 0.5
      move.angular.z = 0

  if data.data <= 55:
      move.linear.x = -0.5
      move.angular.z = 0

  if data.data <= 10:
      move.linear.x = 0
      move.angular.z = 0

  pub.publish(move)

    
  

rospy.init_node('sub_node')
sub = rospy.Subscriber('/cbt', Float64, callback) #We subscribe to the laser's topic
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
rate = rospy.Rate(10)
move = Twist()


rospy.spin()