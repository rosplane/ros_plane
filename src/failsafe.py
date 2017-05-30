#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from fcu_common.msg import Status

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
		if not RTHNOW:
			RTH.data = False
		if not TERMINATENOW:
			Terminate.data = False
		new_failsafe = True

def RTHNOW_callback(msg):
    print"RTH NOW callback"
    RTHNOW = msg.data

def TERMINATENOW_callback(msg):
	TERMINATENOW = msg.data

new_failsafe = True
new_failsafe_time = 0
RTHNOW = False
TERMINATENOW = False

RTH = Bool()
RTH.data = False
Terminate = Bool()
Terminate.data = False

status_sub = rospy.Subscriber('status', Status, failsafe_callback)
RTHNOW_sub = rospy.Subscriber('RTH_now', Bool, RTHNOW_callback)
TERMINATENOW_sub = rospy.Subscriber('TERMINATE_now', Bool, TERMINATENOW_callback)
RTH_pub = rospy.Publisher('RTH', Bool, queue_size=10)
Terminate_pub = rospy.Publisher('terminate_flight', Bool, queue_size=10)


rospy.init_node('Status Watcher')
r = rospy.Rate(10) # 10hz
while not rospy.is_shutdown():
	if RTHNOW:
		RTH.data = True
	if TERMINATENOW:
		Terminate.data = True
	RTH_pub.publish(RTH)
	Terminate_pub.publish(Terminate)
	r.sleep()
