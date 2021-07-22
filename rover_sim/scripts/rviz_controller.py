import rospy
import os
from std_msgs.msg import Bool

def restart_rviz(battery_over):
    if battery_over.data == True:
        os.system("rosrun map_server map_saver -f latest_map")
        os.system("killall -9 rviz")
        os.system("roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=/home/failedmesh/latest_map.yaml")

if __name__ == '__main__':
    rospy.init_node('rviz_controller')
    check_battery = rospy.Subscriber('out_of_battery', Bool, restart_rviz)
    rospy.spin()