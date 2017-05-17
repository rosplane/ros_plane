#!/usr/bin/env python

import RRT
import rospy
import numpy as np

# def main(args):

print('here')

rospy.init_node('rrt_node')

current_pos = np.array([[0.0,0.0,-100.0]]).T
chi_s = 0

goal_pos = np.array([[750,750,-100]]).T
chi_goal = 0

#waypoints are [N,E,D,chi,Va]
wpp_start = [current_pos,chi_s,35]
wpp_end = [goal_pos,chi_goal,35]
R_min = 150

# print(wpp_start[0:3])
map_width = 1000
area_map = np.zeros([map_width,map_width])
for i in range(400,600):
    area_map[i][i] = 200

rate = rospy.Rate(50)
while 1:
    rrt = RRT.RRT(wpp_start, wpp_end, R_min, area_map)
    path = rrt.find_path()
    print(path)
    rate.sleep()


# if __name__ == '__main__':
#
#     main(sys.argv)
