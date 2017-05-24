#!/usr/bin/env python
# Python implementation of "path_planner.cpp"

import rospy
from ros_plane.msg import Waypoint
import math
import sys

num_waypoints = 6

def publishwaypoints():

	# Init ROS Node
	rospy.init_node('landing_path_planner', anonymous=True)

	# Init Publisher
	waypointPublisher = rospy.Publisher('waypoint_path',Waypoint, queue_size=10)

	# Sleep, (this fixed bug of first waypoint not publishing)
	d = rospy.Duration(.5)
	rospy.sleep(d)

	land_point = [float(sys.argv[1]), float(sys.argv[2])] # North, East (meters)
	approach_angle = float(sys.argv[3]) # chi in radians (East of North)

	land = Waypoint()

	# Setup land waypoint
	land.w[0] = land_point[0]
	land.w[1] = land_point[1]
	land.w[2] = 0.0
	land.chi_d = approach_angle
	land.Va_d = 0.0

	land.chi_valid = True # True
	land.set_current = False
	land.reset = False
	land.land = True # True to land now

	# Setup approach 1
	approach1 = Waypoint()
	approach1.w[0] = land_point[0] - math.cos(approach_angle)*20
	approach1.w[1] = land_point[0] - math.sin(approach_angle)*20
	approach1.w[2] = -10.0
	land.chi_d = approach_angle
	approach1.Va_d = 5.0
	land.chi_valid = True # True
	land.set_current = False
	land.reset = False
	approach1.land = False

	# Setup approach 2
	approach2 = Waypoint()
	approach2.w[0] = land_point[0] - math.cos(approach_angle)*40
	approach2.w[1] = land_point[0] - math.sin(approach_angle)*40
	approach2.w[2] = -20.0
	land.chi_d = approach_angle
	approach2.Va_d = 10.0
	land.chi_valid = True # True
	land.set_current = False
	land.reset = False
	approach2.land = False

	# Setup approach 3
	approach3 = Waypoint()
	approach3.w[0] = land_point[0] - math.cos(approach_angle)*60
	approach3.w[1] = land_point[0] - math.sin(approach_angle)*60
	approach3.w[2] = -30.0
	land.chi_d = approach_angle
	approach3.Va_d = 15.0
	land.chi_valid = True # True
	land.set_current = False
	approach3.reset = True
	approach3.land = False

	waypoints = [approach3, approach2, approach1, land]
	# print land.Va_d, approach1.Va_d, approach3.Va_d

    # Loop through each waypoint
	for wp in waypoints:

		# Publish the Waypoint
		waypointPublisher.publish(wp)

		# Sleep
		d = rospy.Duration(0.5)
		rospy.sleep(d)


if __name__ == '__main__':

	# Just run the publisher once
	try:
		publishwaypoints()
	except rospy.ROSInterruptException:
		pass
