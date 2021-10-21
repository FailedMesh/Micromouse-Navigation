import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from rover_sim.srv import distance, distanceResponse
import numpy as np
import math

kP_dist = 0.1
kP_theta = 0.5

def move(request):
    global pos_x, pos_y, kP, yaw
    print("got request")
    target_dist = request.dist
    target_angle = request.angle*math.pi/180
    target_x = target_dist*np.cos(target_angle) + pos_x
    target_y = target_dist*np.sin(target_angle) + pos_y
    dist_to_cover = math.sqrt((target_x - pos_x)**2 + (target_y - pos_y)**2)
    print(dist_to_cover, target_angle)
    command = Twist()
    r = rospy.Rate(10)

    target_rad = target_angle
    print(target_rad, yaw)

    #Treat the discontinuity:
    if (abs(target_rad - yaw) > 0.025):
        if abs(target_rad - yaw) > 3.1416:
            if target_rad > yaw:
                target_rad = target_rad - 1.5708
                yaw = yaw + 4.712389
            elif yaw > target_rad:
                yaw = yaw - 1.5708
                target_rad = target_rad + 4.712389

    while (not rospy.is_shutdown()):
        print(dist_to_cover)
        dist_to_cover = math.sqrt((target_x - pos_x)**2 + (target_y - pos_y)**2)
        command.linear.x = kP_dist*(dist_to_cover)

        if round(pos_x, 3) != round(target_x, 3):
            target_rad = np.arctan2((target_y - pos_y), (target_x - pos_x))
        elif target_y > pos_y:
            target_rad = math.pi/2
        else:
            target_rad = -math.pi/2
        print(target_rad, yaw)

        command.angular.z = kP_theta*(target_rad - yaw)
        if dist_to_cover < 0.02:
            command.linear.x = 0.0
            command.angular.z = 0.0
            pub.publish(command)
            return distanceResponse(True)
        
        pub.publish(command)
        r.sleep()

def get_position(msg):
    global pos_x, pos_y, yaw
    pos_x = msg.pose.pose.position.x
    pos_y = msg.pose.pose.position.y
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)

if __name__ == '__main__':

    rospy.init_node('move_linear')
    sub = rospy.Subscriber ('/odom', Odometry, get_position)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    move_linear_server = rospy.Service('move_linear', distance, move)
    rospy.spin()