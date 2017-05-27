#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool

def failsafe_callback(msg):
	if msg.failsafe: # If in Failsafe mode
			if new_failsafe:
				new_failsafe_time = msg.header.stamp
				new_failsafe = False
			else:
				time = new_fail_safe_time - msg.header.stamp
				if time > 30: # If in failsafe for more than 30s
					RTH.data = True
				if time > 180: # If in failsafe for 3 mins
					Terminate.data = True
	else:
		RTH.data = False
		Terminate.data = False
		new_failsafe = True

new_failsafe = True
new_failsafe_time = 0

status_sub = rospy.Subscriber('status', Status, failsafe_callback)
RTH_pub = rospy.Publisher('RTH', Bool, queue_size=10)
Terminate_pub = rospy.Publisher('terminate_flight', Bool, queue_size=10)

RTH = Bool()
RTH.data = False
Terminate = Bool()
Terminate.data = False

rospy.init_node('Status Watcher')
r = rospy.Rate(10) # 10hz
while not rospy.is_shutdown():
   RTH_pub.publish(RTH)
   Terminate_pub.publish(Terminate)
   r.sleep()