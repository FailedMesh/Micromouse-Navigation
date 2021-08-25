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


class class1(object):

    def __init__(self):
        self.move = Twist()
        self.curr_x = 0.0
        self.curr_y = 0.0
        self.theta = 0.1
        
    def callback3(self, msg):
        self.curr_x = msg.pose.pose.position.x
        self.curr_y = msg.pose.pose.position.y

        rot_q = msg.pose.pose.orientation
        (roll, pitch, self.theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    
    def vel_pub(self,path):
        s = path.shape() 
        for i in range(0,s[0]-1):
            x_1 = path[i+1,0]
            y_1 = path[i+1,1]
            subw = rospy.Subscriber("/odom", Odometry, self.callback3)
            x_err = x_1 - self.curr_x
            y_err = y_1 -self.curr_y
            angle_to_goal = atan2(y_err, x_err)
            if (abs(angle_to_goal - self.theta)>0.1):
                self.move.linear.x = 0.0
                self.move.angular.z = 0.5
            else:
                self.move.linear.x = 0.2

            pub.publish(self.move)
                

rospy.init_node('sub_node')
r = class1()
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
r.vel_pub(path)