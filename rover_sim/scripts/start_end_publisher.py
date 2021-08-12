import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D

def receive_odometry(odometry):
    global started
    global start_pub, end_pub
    x = odometry.pose.pose.position.x
    y = odometry.pose.pose.position.y
    quaternion = odometry.pose.pose.orientation
    _, _, theta = tf.transformations.euler_from_quaternion((quaternion.x, quaternion.y, 
                                                            quaternion.z, quaternion.w))
    current_pose = Pose2D()
    current_pose.x = x
    current_pose.y = y
    current_pose.theta = theta

    if (not started):
        print(current_pose)
        while end_pub.get_num_connections() == 0:
            continue
        end_pub.publish(current_pose)
        print(started)
        started = True
    else:
        start_pub.publish(current_pose)


if __name__ == '__main__':
    rospy.init_node("start_end_publisher")
    start_pub = rospy.Publisher('/start_coordinate', Pose2D, queue_size = 10)
    end_pub = rospy.Publisher('/end_coordinate', Pose2D, queue_size = 10)
    started = False
    odom = rospy.Subscriber('/odom', Odometry, receive_odometry)
    rospy.spin()