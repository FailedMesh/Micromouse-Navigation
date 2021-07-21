#! /usr/bin/env python3
import rospy
from std_msgs.msg import Float64

bt_volt =12
bt_cap = 4.4
dod = 80

mtr_volt = 12
no_load_speed = 1360
no_load_current = 0.01
mtr_const = 0.0123
tor_const = 81.3
rest = 24

rover_speed = 0.5
radius = 0.025

torq_out = (mtr_const/rest)*(mtr_volt - (rover_speed*mtr_const/radius))
curr_req = no_load_current + (torq_out/tor_const)
power_req = mtr_volt*curr_req
bt_cap = bt_cap * dod/100
curr_cap = bt_cap
prt_rem = 100

while prt_rem > 0:
    curr_cap = curr_cap - (curr_req*3600/3600)
    prt_rem = int(curr_cap*100/bt_cap)
    print("Battery remaining = ", prt_rem)
    rospy.init_node('battery_publisher')
    pub = rospy.Publisher('battery', Float64, queue_size=10)
    rate = rospy.Rate(10)
    curr = Float64() # defining the way we can allocate the values
    curr = prt_rem
    pub.publish(curr)
    rate.sleep()


