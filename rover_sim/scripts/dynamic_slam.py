import rospy
import tf
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class explorer():

    def __init__(self):
        self.move = Twist()
        self.x = 0
        self.y = 0
        self.theta = 0.1
        self.MaxObstacleDistance = 0.5
        self.battery = 0

    def update_battery(self, battery_left):
        print("Battery remaining: ", battery_left.data)
        self.battery = battery_left.data

    def update_position(self, odometry):
        self.x = odometry.pose.pose.position.x
        self.y = odometry.pose.pose.position.y
        quaternion = [ odometry.pose.pose.orientation.x,
                        odometry.pose.pose.orientation.y,
                        odometry.pose.pose.orientation.z,
                        odometry.pose.pose.orientation.w
                        ]
        _, _, self.theta = tf.transformations.euler_from_quaternion(quaternion)

    def avoid_obstacle(self, laserscan):

        if (laserscan.ranges[300] > self.MaxObstacleDistance) and (self.battery > 55.0):
            self.move.linear.x = 0.2
            self.move.angular.z = 0.0

        elif (laserscan.ranges[300] <= self.MaxObstacleDistance) and (self.battery > 55.0): 
            self.move.linear.x = 0.0
            self.move.angular.z = 0.5

        else:
            print("ENOUGH EXPLORING, NEED TO GET BACK!!")
            self.move.linear.x = 0.0
            self.move.linear.y = 0.0
            self.move.angular.z = 0.0


if __name__ == '__main__':
    rospy.init_node('dynamic_slam')
    turtle_bot = explorer()
    rate = rospy.Rate(10)
    battery_sub = rospy.Subscriber('battery', Float64, turtle_bot.update_battery)
    odometry_sub = rospy.Subscriber('odom', Odometry, turtle_bot.update_position)
    laser_sub = rospy.Subscriber('scan', LaserScan, turtle_bot.avoid_obstacle)
    velocity_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(1)
    velocity_pub.publish(turtle_bot.move)
    rate.sleep()
    rospy.spin()