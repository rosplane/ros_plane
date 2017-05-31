#!/usr/bin/env python

import rospy
from rosflight_msgs.msg import State
from ros_plane.msg import Dubin, Waypoint, Current_Path
from std_msgs.msg import Float32

import numpy as np
from math import *
# import matplotlib.pyplot as plt
# import matplotlib.animation as animation
# from matplotlib.lines import Line2D
import pygame
from pygame.locals import *

import rrt_path_planner


class flight_plots(): # will only extract x,y data for plotting

	def __init__(self):
		#----------------------Params-------------------------------
		self.x_size = 650
		self.y_size = 650
		self.north_max = 2000
		self.east_max = 2000
		self.pn = -(self.y_size/2)*(0.0/self.north_max) + self.y_size/2
		self.pe = (self.x_size/2)*(0.0/self.east_max) + self.x_size/2
		self.wp_n = 20000
		self.wp_e = 20000

		# Current Path stuff
		self.path_flag = False
		self.r = [-999, -999, -999]
		self.q = [-999, -999, -999]
		self.c = [-999, -999, -999]
		self.rho = 10.0
		self.lam = 0

		self.dubin = Dubin()
		self.dubin_valid = False

		self.new_error = False
		self.error = 0.0
		#------------------------------------------------------------

		# ROS Subscribers
		rospy.Subscriber("/junker/truth", State, self.callback)
		rospy.Subscriber("current_path", Current_Path, self.path_callback)
		rospy.Subscriber("current_waypoint", Waypoint, self.wp_callback)
		rospy.Subscriber("dubin_params", Dubin, self.dubin_callback)
		rospy.Subscriber("waypoint_error", Float32, self.error_callback)

		# pygame init
		pygame.init()
		self.screen=pygame.display.set_mode([self.x_size, self.y_size])
		pygame.display.set_caption('ROSplane plots')
		self.white = 255, 240, 200
		self.black = 20, 20, 40
		self.red = 255, 0, 0
		self.green = 0, 255, 0
		self.blue = 0, 0, 255
		self.bg = 0, 255, 255
		self.font = pygame.font.Font(None,15)
		self.screen.fill(self.white)
		pygame.draw.line(self.screen,self.black,[0,self.y_size/2],[self.x_size,self.y_size/2])
		pygame.draw.line(self.screen,self.black,[self.x_size/2,0],[self.x_size/2,self.y_size])
		pygame.display.update()


	def callback(self, msg):
		# self.pn = FW_State.position[0]
		# self.pe = FW_State.position[1]
		self.pn = -(self.y_size/2)*(msg.position[0]/self.north_max) + self.y_size/2
		self.pe = (self.x_size/2)*(msg.position[1]/self.east_max) + self.x_size/2

	def path_callback(self,msg):
		self.path_flag = msg.flag
		self.r = msg.r
		self.q = msg.q
		self.c = msg.c
		self.rho = msg.rho
		# self.lam = msg.lamd

	def wp_callback(self,msg):
		self.wp_n = -(self.y_size/2)*(msg.w[0]/self.north_max) + self.y_size/2
		self.wp_e = (self.x_size/2)*(msg.w[1]/self.east_max) + self.x_size/2

	def dubin_callback(self,msg):
		self.dubin = msg
		self.dubin_valid = True

	def error_callback(self,msg):
		self.error = np.float16(msg.data)
		self.new_error = True

	def print_states(self):
		print "pn: ", self.pn
		print "pe: ", self.pe

	def map(self):
		# Obstacels = Lines and Circles
		square = [400.0, 400.0, 600.0, 600.0] # corners
		sq_n1 = -(self.y_size/2)*(square[0]/self.north_max) + self.y_size/2
		sq_e1 = (self.x_size/2)*(square[1]/self.east_max) + self.x_size/2

		sq_n2 = -(self.y_size/2)*(square[2]/self.north_max) + self.y_size/2
		sq_e2 = (self.x_size/2)*(square[3]/self.east_max) + self.x_size/2

		pointlist1 = [[sq_e1,sq_n1],[sq_e1,sq_n2],[sq_e2,sq_n2],[sq_e2,sq_n1]]

		buf = 50
		sq_n1b = -(self.y_size/2)*((square[0]-buf)/self.north_max) + self.y_size/2
		sq_e1b = (self.x_size/2)*((square[1]-buf)/self.east_max) + self.x_size/2

		sq_n2b = -(self.y_size/2)*((square[2]+buf)/self.north_max) + self.y_size/2
		sq_e2b = (self.x_size/2)*((square[3]+buf)/self.east_max) + self.x_size/2

		pointlist2 = [[sq_e1b,sq_n1b],[sq_e1b,sq_n2b],[sq_e2b,sq_n2b],[sq_e2b,sq_n1b]]

		square = [-400.0, -400.0, -600.0, -600.0] # corners
		sq_n1 = -(self.y_size/2)*(square[0]/self.north_max) + self.y_size/2
		sq_e1 = (self.x_size/2)*(square[1]/self.east_max) + self.x_size/2

		sq_n2 = -(self.y_size/2)*(square[2]/self.north_max) + self.y_size/2
		sq_e2 = (self.x_size/2)*(square[3]/self.east_max) + self.x_size/2

		pointlist3 = [[sq_e1,sq_n1],[sq_e1,sq_n2],[sq_e2,sq_n2],[sq_e2,sq_n1]]

		buf = 50
		sq_n1b = -(self.y_size/2)*((square[0]+buf)/self.north_max) + self.y_size/2
		sq_e1b = (self.x_size/2)*((square[1]+buf)/self.east_max) + self.x_size/2

		sq_n2b = -(self.y_size/2)*((square[2]-buf)/self.north_max) + self.y_size/2
		sq_e2b = (self.x_size/2)*((square[3]-buf)/self.east_max) + self.x_size/2

		pointlist4 = [[sq_e1b,sq_n1b],[sq_e1b,sq_n2b],[sq_e2b,sq_n2b],[sq_e2b,sq_n1b]]

		return [pointlist1,pointlist2, pointlist3, pointlist4]

	def animate(self):
		self.screen.fill(self.black)
		pygame.draw.line(self.screen,self.white,[0,self.y_size/2],[self.x_size,self.y_size/2])
		pygame.draw.line(self.screen,self.white,[self.x_size/2,0],[self.x_size/2,self.y_size])

		# Position
		pygame.draw.circle(self.screen, self.blue, (int(self.pe),int(self.pn)),5)

		# Current Waypoint
		pygame.draw.circle(self.screen, self.green, (int(self.wp_e),int(self.wp_n)),5)

		# Draw Obstacles
		pointlist = self.map()
		pygame.draw.polygon(self.screen,self.red,pointlist[0])
		pygame.draw.polygon(self.screen,self.red,pointlist[1],3)
		pygame.draw.polygon(self.screen,self.red,pointlist[2])
		pygame.draw.polygon(self.screen,self.red,pointlist[3],3)

		# Draw Dubin Params
		if self.dubin_valid:
			#start
			cs_n = int(-(self.y_size/2)*(self.dubin.cs[0]/self.north_max) + self.y_size/2)
			cs_e = int((self.x_size/2)*(self.dubin.cs[1]/self.east_max) + self.x_size/2)
			rad = int(0.5*self.x_size*(self.dubin.R/self.north_max))
			pygame.draw.circle(self.screen, self.bg, (int(cs_e),int(cs_n)),rad, 1)

			#line
			start_n = int(-(self.y_size/2)*(self.dubin.w1[0]/self.north_max) + self.y_size/2)
			start_e = int((self.x_size/2)*(self.dubin.w1[1]/self.east_max) + self.x_size/2)
			end_n = int(-(self.y_size/2)*(self.dubin.w2[0]/self.north_max) + self.y_size/2)
			end_e = int((self.x_size/2)*(self.dubin.w2[1]/self.east_max) + self.x_size/2)
			pygame.draw.line(self.screen, self.bg, [start_e,start_n], [end_e,end_n])

			#end
			ce_n = int(-(self.y_size/2)*(self.dubin.ce[0]/self.north_max) + self.y_size/2)
			ce_e = int((self.x_size/2)*(self.dubin.ce[1]/self.east_max) + self.x_size/2)
			rad = int(0.5*self.x_size*(self.dubin.R/self.north_max))
			pygame.draw.circle(self.screen, self.bg, (int(ce_e),int(ce_n)),rad, 1)

		# Current Path
		if self.path_flag == False and (self.rho != 0):
			# Circle Path
			center_n = int(-(self.y_size/2)*(self.c[0]/self.north_max) + self.y_size/2)
			center_e = int((self.x_size/2)*(self.c[1]/self.east_max) + self.x_size/2)
			center = (center_e,center_n)
			rad = int(0.5*self.x_size*(self.rho/self.north_max))
			pygame.draw.circle(self.screen, self.red, center, rad, 1)
		if self.path_flag == True:
			# Straight Path
			start_n = int(-(self.y_size/2)*(self.r[0]/self.north_max) + self.y_size/2)
			start_e = int((self.x_size/2)*(self.r[1]/self.east_max) + self.x_size/2)
			end_n = int(250*-self.q[0])+start_n
			end_e = int(250*self.q[1])+start_e
			pygame.draw.line(self.screen, self.red, [start_e,start_n], [end_e,end_n])

		if True: #self.new_error:
			scoretext = self.font.render('Waypoint Error: '+str(self.error),1,self.red)
			self.screen.blit(scoretext,(self.x_size - 150,self.y_size-25))

		pygame.display.update()
		# print "animate"


##############################
#### Main Function to Run ####
##############################
if __name__ == '__main__':
	# Initialize Node
	rospy.init_node('flight_dyn_plots')

	# set rate
	hz = 1.0
	rate = rospy.Rate(hz)

	# init path_manager_base object
	ploter = flight_plots()

	# Loop
	while not rospy.is_shutdown():
		ploter.animate()
		rate.sleep()
