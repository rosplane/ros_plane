#!/usr/bin/env python
# Python implementation of "path_planner.cpp"

import rospy
from ros_plane.msg import Waypoint
import math
import RRT
import numpy as np

# num_waypoints = 6

def publishwaypoints():

	# Init ROS Node
	rospy.init_node('ros_plane_path_planner', anonymous=True)

	# Init Publisher
	waypointPublisher = rospy.Publisher('waypoint_path',Waypoint, queue_size=10)

	# Sleep, (this fixed bug of first waypoint not publishing)
	d = rospy.Duration(.5)
	rospy.sleep(d)
	# Set waypoints
	Va = 30.0#8.5 # 11.0
	wps =  [
				0, 0, -75, 0.0, Va,
				750, 0, -75, 0*math.pi/180.0, Va,
				0, 1000, -75, -math.pi, Va,
				-1000, 0, -75, -math.pi/2.0, Va,
				0, -1000, -75, 0.0, Va,
				1000, 0, -75, math.pi/2, Va]
				# -10, -10, -30, -45, Va,
				# -10, -125, -30, -135*math.pi/180, Va,
				# -125, -10, -30, 45*math.pi/180, Va,
				# -125, -125, -30, 135*math.pi/180, Va]
	num_waypoints = len(wps)/5

	# Params
	R_min = 200

	# print(wpp_start[0:3])
	map_width = 3000
	area_map = np.zeros([map_width,map_width])
	buf = 50
	half_map = map_width / 2
	for i in range(500-buf,1100+buf):
		for j in range(500-buf,1100+buf):
			area_map[i][j] = 200
	for i in range(400+half_map-buf,1000+half_map+buf):
		for j in range(400+half_map-buf,1000+half_map+buf):
			area_map[i][j] = 200

	add_node = []
	add_index = []
	wps_rrt = wps

	for i in range(0,num_waypoints-1):
		wpp_pos = np.array([[wps[i*5+0],wps[i*5+1],wps[i*5+2]]]).T
		wpp_start = [wpp_pos, wps[i*5+3], wps[i*5+4]]
		start_node = RRT.rrtNode(0,None,wpp_start)

		wpp_pos = np.array([[wps[i*5+5+0],wps[i*5+5+1],wps[i*5+5+2]]]).T
		wpp_end = [wpp_pos, wps[i*5+5+3], wps[i*5+5+4]]
		end_node = RRT.rrtNode(0, None, wpp_end)
		rr = RRT.RRT(wpp_start, wpp_end, R_min, area_map)
		path_valid = rr.check_dubins_path(start_node,end_node)

		if not path_valid:
			print "Collision on index: ", i
			t_past = rospy.Time.now()
			path = rr.find_path()
			rospy.logwarn(rospy.Time.now()-t_past)

			print len(path)-2
			for k in range(0, len(path)-2):
				index = len(path) - 2 - k
				for _ in range(0,len(path[index])):
					wps_rrt.insert(i*5+5,path[index][len(path[index])-1-_])

			print path
			print wps_rrt
		else:
			print 'OK index: ', i


	num_rrt_wps = len(wps_rrt)/5
	# Loop through each waypoint
	for i in range(0,num_rrt_wps):

		# Make waypoint a Waypoint msg
		new_waypoint = Waypoint()

		new_waypoint.w[0] = wps_rrt[i*5 + 0]
		new_waypoint.w[1] = wps_rrt[i*5 + 1]
		new_waypoint.w[2] = wps_rrt[i*5 + 2]
		new_waypoint.chi_d = wps_rrt[i*5 + 3]

		new_waypoint.chi_valid = True # True
		new_waypoint.set_current = False
		new_waypoint.Va_d = wps_rrt[i*5 + 4]

		# Publish the Waypoint
		waypointPublisher.publish(new_waypoint)
		rospy.logwarn('waypoint published')

		# Sleep
		d = rospy.Duration(0.5)
		rospy.sleep(d)


if __name__ == '__main__':
	# Just run the publisher once
	try:
		publishwaypoints()
	except rospy.ROSInterruptException:
		pass
