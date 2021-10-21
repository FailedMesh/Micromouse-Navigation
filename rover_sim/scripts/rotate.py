import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from rover_sim.srv import angle, angleResponse
import math

roll = pitch = yaw = 0.0
target_angle = 9
kP = 0.5

def rotate(request):
    global yaw, kP
    print("got request")
    target_angle = request.angle
    #print(target_angle, yaw)
    command = Twist()
    r = rospy.Rate(10)
    while (not rospy.is_shutdown()):
        target_rad = target_angle * math.pi/180
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


        command.angular.z = kP*(target_rad - yaw)
        if abs(target_rad - yaw) < 0.005:
            command.angular.z = 0.0
            pub.publish(command)
            return angleResponse(True)
        pub.publish(command)
        r.sleep()

def get_rotation (msg):
    global roll, pitch, yaw
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)

if __name__ == '__main__':

    rospy.init_node('rotation')
    sub = rospy.Subscriber ('/odom', Odometry, get_rotation)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rotation_server = rospy.Service('rotation', angle, rotate)
    rospy.spin()