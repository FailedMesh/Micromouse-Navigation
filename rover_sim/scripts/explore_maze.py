from numpy.core.numeric import Inf
import os
import math
import time

from numpy.lib.function_base import angle
import rospy
import tf
import numpy as np
from std_msgs.msg import Float64, Bool
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
from rover_sim.srv import angle
from rover_sim.srv import distance

def rightRotate(lists, num):
    output_list = []
      
    # Will add values from n to the new list
    for item in range(len(lists) - num, len(lists)):
        output_list.append(lists[item])
      
    # Will add the values before
    # n to the end of new list    
    for item in range(0, len(lists) - num): 
        output_list.append(lists[item])
          
    return output_list

class explorer():

    def __init__(self):
        self.active = False
        self.origin = Point()
        self.origin.z = 0.0
        self.move = Twist()
        self.x = 0
        self.y = 0
        self.theta = 0.1
        self.orientation = 0
        self.MaxObstacleDistance = 0.8
        self.out_of_battery = Bool()
        self.out_of_battery = False
        self.published_battery = False

    def update_position(self, odometry):
        self.x = odometry.pose.pose.position.x
        self.y = odometry.pose.pose.position.y
            
        quaternion = [ odometry.pose.pose.orientation.x,
                        odometry.pose.pose.orientation.y,
                        odometry.pose.pose.orientation.z,
                        odometry.pose.pose.orientation.w
                        ]
        
        if self.active == False:
            self.origin.x = self.x
            self.origin.y = self.y
            self.active = True

        _, _, self.theta = tf.transformations.euler_from_quaternion(quaternion)



    def navigate(self, laserscan):


        #Scan 4 directions:
        top_scan = np.float32(laserscan.ranges[-5:] + laserscan.ranges[0:6])
        right_scan = np.float32(laserscan.ranges[85:96])
        bottom_scan = np.float32(laserscan.ranges[175:186])
        left_scan = np.float32(laserscan.ranges[265:276])

        #Resolve:
        resolvation = (np.float32(np.arange(-5,6))*math.pi)/180
        resolvation = np.cos(resolvation).T
        top = np.dot(top_scan, resolvation)/10
        right = np.dot(right_scan, resolvation)/10
        bottom = np.dot(bottom_scan, resolvation)/10
        left = np.dot(left_scan, resolvation)/10

        #Encode the 4 directions into an array, 1 for wall, 0 for no wall
        point_data = [0, 0, 0, 0]

        #Check with threshold if there is a wall or not
        if top < 0.6 :
            point_data[0] = 1
        if right < 0.6:
            point_data[1] = 1
        if bottom < 0.6:
            point_data[2] = 1
        if left < 0.6:
            point_data[3] = 1

        if self.orientation == 0:
            heading = (self.x, 1)
        elif self.orientation == 90:
            heading = (self.y, -1)
        elif self.orientation == -180:
            heading = (self.x, -1)
        elif self.orientation == -90:
            heading = (self.y, 1)

        #Check orientation and transform point_data accordingly:
        #point_data_map = rightRotate(point_data, self.orientation)
        #print(point_data_map)

        #Take manual keyboard input for movement of bot:
        correct_command = False
        while not correct_command:
            move_command = input("Enter movement command (W, A, S, D, Q, E): ")
            if move_command in ['W', 'A', 'S', 'D', 'Q', 'E', 'w', 'a', 's', 'd', 'q', 'e']:
                correct_command = True
            else:
                print("Invalid command, try again\n")

        #Wait for movement services:
        rospy.wait_for_service('rotation')
        rospy.wait_for_service('move_linear')

        #Go straight 1 metre:
        if move_command in ['Q', 'q']:
            try:
                forward = rospy.ServiceProxy('move_linear', distance)
                output = forward(1, self.orientation)
            except rospy.ServiceException as e:
                print("Service Failed: ", e)

        #Go back 1 metre:
        if move_command in ['E', 'e']:
            try:
                backward = rospy.ServiceProxy('move_linear', distance)
                output = backward(-1, self.orientation)
            except rospy.ServiceException as e:
                print("Service Failed: ", e)

        #Look top:
        if move_command in ['W', 'w']:
            try:
                rotate = rospy.ServiceProxy('rotation', angle)
                output = rotate(45)
                self.orientation = 0
            except rospy.ServiceException as e:
                print("Service Failed: ", e)

        #Look right:
        if move_command in ['D', 'd']:
            try:
                rotate = rospy.ServiceProxy('rotation', angle)
                output = rotate(-90)
                self.orientation = -90
            except rospy.ServiceException as e:
                print("Service Failed: ", e)

        #Look down:
        if move_command in ['S', 's']:
            try:
                rotate = rospy.ServiceProxy('rotation', angle)
                output = rotate(-180)
                self.orientation = -180
            except rospy.ServiceException as e:
                print("Service Failed: ", e)

        #Look left:
        if move_command in ['A', 'a']:
            try:
                rotate = rospy.ServiceProxy('rotation', angle)
                output = rotate(90)
                self.orientation = 90
            except rospy.ServiceException as e:
                print("Service Failed: ", e)


        theta = self.theta

#        if Inf in top_scan:
#            self.move.linear.x = 0.1
#            self.move.angular.z = 0.0
#            print("Straight")

#        elif Inf not in laserscan.ranges:
#            target_angle = np.where(laserscan.ranges == np.amax(laserscan.ranges))
#            target_angle = math.pi*target_angle/180
#            if (abs(target_angle - theta) > 0.025):
#                if abs(target_angle - theta) > 3.1416:
#                    if target_angle > theta:
#                        target_angle = target_angle - 1.5708
#                        theta = theta + 4.712389
#                    elif theta > target_angle:
#                        theta = theta - 1.5708
#                        target_angle = target_angle + 4.712389
#
#                if target_angle > theta:
#                    self.move.linear.x = 0.0
#                    self.move.angular.z = 0.2
#                else:
#                    self.move.linear.x = 0.0
#                    self.move.angular.z = -0.2

#        if Inf not in top_scan:
#            os.system("rosrun map_server map_saver -f /home/failedmesh/catkin_ws/src/Micromouse-Navigation/rover_sim/scripts/latest_map")
#            
#
#        else:
#            self.move.linear.x = 0.0
#            self.move.angular.z = 0.2
#            print("Turn")

        print(self.move.linear.x, " ", self.move.linear.y, " ", self.move.angular.z)
        #velocity_pub.publish(self.move)


if __name__ == '__main__':
    rospy.init_node('dynamic_slam')
    turtle_bot = explorer()
    rate = rospy.Rate(10)

    #velocity_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    odometry_sub = rospy.Subscriber('odom', Odometry, turtle_bot.update_position)
    laser_sub = rospy.Subscriber('scan', LaserScan, turtle_bot.navigate)
    rate = rospy.Rate(1)

    rate.sleep()
    rospy.spin()