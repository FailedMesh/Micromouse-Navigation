#! /usr/bin/env python3

from numpy.core.fromnumeric import take
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan 
from tf.transformations import euler_from_quaternion
from math import atan2, cos, pi, sin, sqrt
import roslaunch

class class1(object):
    def __init__(self):
        self.move = Twist()
        self.bt = 100
        self.curr_x = 0.0
        self.curr_y = 0.0
        self.disToObstacle = 0.5
        self.theta = 0.1
        

    def callback3(self, msg):
        self.curr_x = msg.pose.pose.position.x
        self.curr_y = msg.pose.pose.position.y

        rot_q = msg.pose.pose.orientation
        (roll, pitch, self.theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

    def callback(self, data): 
        # rospy.loginfo(" Battery remaining is -  %s",data.data)
        self.bt = data.data  

    def callback2(self, msg):
        rate = rospy.Rate(10)
        goal = Point()
        goal.x = 0
        goal.y = 0
        x_err = goal.x - self.curr_x
        y_err = goal.y -self.curr_y
        angle_to_goal = atan2(y_err, x_err)
        d_goal = sqrt((x_err*x_err) + (y_err*y_err))

#If the distance to an obstacle in front of the robot is bigger than 1 meter, the robot will move forward
        if (msg.ranges[1] > self.disToObstacle) and (self.bt > 55):
            self.move.linear.x = 0.2
            self.move.angular.z = 0.0

#If the distance to an obstacle in front of the robot is smaller than 1 meter, the robot will turn left
        if ((msg.ranges[1] <= self.disToObstacle) and (self.bt > 55)) or ((msg.ranges[355] <= self.disToObstacle) and (self.bt > 55)) or ((msg.ranges[6] <= self.disToObstacle) and (self.bt > 55)): 
            self.move.linear.x = 0.0
            self.move.angular.z = 0.5
            rospy.loginfo(" Battery remaining is -  %s",self.bt)

        if (msg.ranges[1] > self.disToObstacle) and (self.bt > 55):
            self.move.linear.x = 0.2
            self.move.angular.z = 0.0
            rospy.loginfo(" Battery remaining is -  %s",self.bt)
            

        if (self.bt <= 55) and (self.bt > 5): 
            goal = Point()
            goal.x = 0
            goal.y = 0
            x_err = goal.x - self.curr_x
            y_err = goal.y -self.curr_y
            angle_to_goal = atan2(y_err, x_err)
            d_goal = sqrt((x_err*x_err) + (y_err*y_err))
            min_value = min(msg.ranges)
            min_index = msg.ranges.index(min_value)
            d_obstacle = 0
            theta_obstacle = 0
            
            theta_goal = angle_to_goal
            if min_value>0 and min_value<=0.5 and min_index <=180:
                theta_obstacle = min_index*pi/180
                d_obstacle = min_value
            if min_value>0 and min_value<=0.5 and min_index >180:
                theta_obstacle = (min_index-360)*pi/180
                d_obstacle = min_value

            rospy.loginfo(" Battery remaining is -  %s",self.bt)
            # rospy.loginfo(" Dist to obstacle %s",d_obstacle)
            phi = atan2(10*d_obstacle*sin(pi+theta_obstacle),(d_goal + (10*d_obstacle*sin(pi+theta_obstacle))))
            #rospy.loginfo(" Phi %s",phi)

            theta2 = phi + angle_to_goal

            if d_goal>0.1 and abs(theta2-self.theta) >= 0.2:
                self.move.angular.z = 0.5*(theta2-self.theta)
                self.move.linear.x = 0.2

            if d_goal>0.1 and abs(theta2-self.theta) < 0.2:
                self.move.angular.z = 0
                self.move.linear.x = 0.2
            
            if d_goal <=0.1:
                self.move.linear.x = 0.0
                self.move.angular.z = 0

            
            # rospy.loginfo('Returning to origin.. Autonomous navigation node up and running')
            # package = 'rover_srv'
            # executable = 'trt.py'
            # node = roslaunch.core.Node(package, executable)

            # launch = roslaunch.scriptapi.ROSLaunch()
            # launch.start()

            # task = launch.launch(node)
           
            
        if (self.bt) <= 5 or ((self.bt) <= 55 and d_goal <=0.1):
            self.move.linear.x = 0
            self.move.angular.z = 0
            rospy.loginfo(" Battery remaining is -  %s",self.bt)

        if ((self.bt) <= 55 and d_goal <=0.1):
            self.move.linear.x = 0
            self.move.angular.z = 0
            rospy.loginfo(" Reached the charging check point -  %s",self.bt)
            

        pub.publish(self.move)
        
        


    
  

rospy.init_node('sub_node')
r = class1()
rate = rospy.Rate(10)
sub = rospy.Subscriber('/cbt', Float64, r.callback) #We subscribe to the laser's topic
subo = rospy.Subscriber('/scan', LaserScan, r.callback2)
subw = rospy.Subscriber("/odom", Odometry, r.callback3)
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)




rospy.spin()
