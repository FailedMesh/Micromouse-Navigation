from numpy.core.fromnumeric import shape, size
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import atan2
from rover_sim.msg import float_array
from rover_sim.msg import path_array
import math

def save_path(data):
    global path, target, i, n
    print("received")
    path = data.path
    print(path)
    target = path[0].point
    i = 0
    n = len(path)

def traverse(msg):
    global path, target, i, n
    if path != None:
        move = Twist()
        curr_x = msg.pose.pose.position.x
        curr_y = msg.pose.pose.position.y
        rot_q = msg.pose.pose.orientation
        (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
        x, y = target[0], target[1]
        x_err = x - curr_x
        y_err = y - curr_y
        angle_to_goal = atan2(y_err, x_err)
        error = math.sqrt((x_err**2) + (y_err**2))
        print("Point number: ", i, "\n", error, "\n", (angle_to_goal - theta), "\n---\n")
        if (abs(angle_to_goal - theta) > 0.025):
            if abs(angle_to_goal - theta) > 3.1416:
                if angle_to_goal > theta:
                    angle_to_goal = angle_to_goal - 1.5708
                    theta = theta + 4.712389
                elif theta > angle_to_goal:
                    theta = theta - 1.5708
                    angle_to_goal = angle_to_goal + 4.712389
            if angle_to_goal > theta:
                move.linear.x = 0.0
                move.angular.z = 0.2
            else:
                move.linear.x = 0.0
                move.angular.z = -0.2
#            else:
#                if angle_to_goal < theta:
#                    move.linear.x = 0.0
#                    move.angular.z = 0.2
#                else:
#                    move.linear.x = 0.0
#                    move.angular.z = -0.2
        elif error > 0.05:
            move.linear.x = 0.2
        else:
            if i == n-1:
                path = None
                print("path traversed")
            else:
                i = i + 1
                target = path[i].point
        pub.publish(move)

       
                
if __name__ == '__main__':
    rospy.init_node('velocity')
    path = None
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    path_sub = rospy.Subscriber('/path', path_array, save_path)
    odom_sub = rospy.Subscriber("/odom", Odometry, traverse)
    rospy.spin()