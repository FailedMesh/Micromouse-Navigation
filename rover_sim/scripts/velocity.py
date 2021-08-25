from os import path
import numpy as np
from numpy.core.fromnumeric import shape, size
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan 
from tf.transformations import euler_from_quaternion
from math import atan2

def callback3(self, msg):
    move = Twist()
    curr_x = msg.pose.pose.position.x
    curr_y = msg.pose.pose.position.y
    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    x_err = x_1 - curr_x
    y_err = y_1 - curr_y
    angle_to_goal = atan2(y_err, x_err)
    if (abs(angle_to_goal - self.theta)>0.1):
        move.linear.x = 0.0
        move.angular.z = 0.5
    else:
        move.linear.x = 0.2
    pub.publish(move)

       
                

rospy.init_node('sub_node')
s = path.shape() 
for i in range(0,s[0]-1):
    x_1 = path[i+1,0]
    y_1 = path[i+1,1]
    subw = rospy.Subscriber("/odom", Odometry, self.callback3)
    
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
