from os import WUNTRACED
import rospy
import tf
from std_msgs.msg import Float64, Bool
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point

class explorer():

    def __init__(self):
        self.active = False
        self.origin = Point()
        self.origin.z = 0.0
        self.move = Twist()
        self.x = 0
        self.y = 0
        self.theta = 0.1
        self.MaxObstacleDistance = 0.5
        self.battery = 0
        self.out_of_battery = Bool()
        self.out_of_battery = False
        self.published_battery = False

    def update_battery(self, battery_left):
        #print("Battery remaining: ", battery_left.data)
        self.battery = battery_left.data

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
            self.out_of_battery = True
            origin_pub.publish(self.origin)

        print(self.move.linear.x, " ", self.move.linear.y, " ", self.move.angular.z)
        velocity_pub.publish(self.move)

        if (not self.published_battery) and self.out_of_battery:
            battery_pub.publish(self.out_of_battery)
            self.published_battery = True


if __name__ == '__main__':
    rospy.init_node('dynamic_slam')
    turtle_bot = explorer()
    rate = rospy.Rate(10)

    velocity_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    battery_pub = rospy.Publisher('/out_of_battery', Bool, queue_size = 10)
    origin_pub = rospy.Publisher('/origin', Point, queue_size = 10)

    battery_sub = rospy.Subscriber('battery', Float64, turtle_bot.update_battery)
    odometry_sub = rospy.Subscriber('odom', Odometry, turtle_bot.update_position)
    laser_sub = rospy.Subscriber('scan', LaserScan, turtle_bot.avoid_obstacle)
    velocity_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(1)

    rate.sleep()
    rospy.spin()