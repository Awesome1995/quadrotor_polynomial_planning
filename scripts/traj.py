#!/usr/bin/env python
import rospy
import numpy as np
from nav_msgs.msg import Path
from std_msgs.msg import Float64, Float64MultiArray
from geometry_msgs.msg import PoseStamped, TwistStamped

import matplotlib.pyplot as pl

# Custom message
from fla.msg import Coeff

import array_utils as au

class gen_traj:
	def __init__(self):

		# Flag to use splines 
		self.splines = True

		# Step size for psuedo distance variable
		self.h = 1.0

		# Initial derivatives for splines algorithm
		self.xp = 0
		self.yp = 0
		self.zp = 0

		# Estimate of arc length
		self.s_e = 0

		# First time receiving WP
		self.first = True

		# Initialize dictionaries
		self.dict_s = {}
		self.dict_x = {}
		self.dict_y = {}
		self.dict_z = {}

		# Arc length delta
		self.dq = 0.001

		self.coeff = Coeff()

		# ROS initialization
		self.geom_props = Float64MultiArray()
		self.wp_sub = rospy.Subscriber('waypoint_list', Path, self.WP_list_CB)
		self.coeff_pub = rospy.Publisher('coeffs',Coeff,queue_size=1)
		rospy.loginfo("Trajecotry Spline Initialized")
		rospy.loginfo("Waiting for waypoint list...")

	def WP_list_CB(self,msg):
		rospy.loginfo("Waypoints received")
		self.data = msg.poses
		now = rospy.get_time()
		self.WP_parse()
		if self.splines:
			if not self.first:
				self.deriv_calc()
			self.spline_coeffs()
		else:
			self.line_coeffs()
		self.calc_arc_leng()
		self.send_coeff()
		then = rospy.get_time()
		print then-now
		if self.first:
			self.first = False

	def send_coeff(self):
		self.seg_len_MA = Float64MultiArray()
		self.coeff_x_MA = Float64MultiArray()
		self.coeff_y_MA = Float64MultiArray()
		self.coeff_z_MA = Float64MultiArray()

		# Convert numpy array to multi array
		au.numpyArray2MultiArray(self.s,self.seg_len_MA)
		au.numpyArray2MultiArray(self.coeff_x,self.coeff_x_MA)
		au.numpyArray2MultiArray(self.coeff_y,self.coeff_y_MA)
		au.numpyArray2MultiArray(self.coeff_z,self.coeff_z_MA)

		self.coeff.s = self.seg_len_MA
		self.coeff.coeff_x = self.coeff_x_MA
		self.coeff.coeff_y = self.coeff_y_MA
		self.coeff.coeff_z = self.coeff_z_MA

		self.coeff_pub.publish(self.coeff)
		
	def WP_parse(self):
		self.N = len(self.data)
		self.wp = [[],[],[]]
		for i in range(self.N):
			self.wp[0].append(self.data[i].pose.position.x)
			self.wp[1].append(self.data[i].pose.position.y)
			self.wp[2].append(self.data[i].pose.position.z)
		self.wp = np.asarray(self.wp)

	def deriv_calc(self):
		flag = False
		i = 0
		q = self.h*(self.N-1)*self.s_e/self.L
		while not flag:
			if self.s_e < self.dict_s[i+1]:
				flag = True
				self.xp = 3*self.ax[i]*q**2 + 2*self.bx[i]*q + self.cx[i]
				self.yp = 3*self.ay[i]*q**2 + 2*self.by[i]*q + self.cy[i]
				self.zp = 3*self.az[i]*q**2 + 2*self.bz[i]*q + self.cz[i]
			else:
				i+=1

	def line_coeffs(self):
		# Psuedo distance variable
		self.q = np.arange(0,self.h*(self.N-1),self.h)

		self.cx = self.wp[0,1:] - self.wp[0,0:-1]
		self.cy = self.wp[1,1:] - self.wp[1,0:-1]
		self.cz = self.wp[2,1:] - self.wp[2,0:-1]

		self.dx = self.wp[0,0:self.N-1] 
		self.dy = self.wp[1,0:self.N-1]
		self.dz = self.wp[2,0:self.N-1]

		self.ax = np.zeros(np.size(self.cx))
		self.ay = np.zeros(np.size(self.cy))
		self.az = np.zeros(np.size(self.cz))

		self.bx = np.zeros(np.size(self.cx))
		self.by = np.zeros(np.size(self.cy))
		self.bz = np.zeros(np.size(self.cz))

		self.coeff_x =  np.hstack((self.ax[None].T,self.bx[None].T,self.cx[None].T,self.dx[None].T))
		self.coeff_y =  np.hstack((self.ay[None].T,self.by[None].T,self.cy[None].T,self.dy[None].T))
		self.coeff_z =  np.hstack((self.az[None].T,self.bz[None].T,self.cz[None].T,self.dz[None].T))


	def spline_coeffs(self):
		# Psuedo distance variable
		self.q = np.arange(0,self.h*(self.N-1),self.h)

		# Coefficient matrix that needs to be inverted
		A = 4*np.eye(self.N) + np.diag(np.ones(self.N-1),1) + np.diag(np.ones(self.N-1),-1)
		A[0,0] = 2
		A[-1,-1] = 1
		A[-1,self.N-2] = 0

		# Initialize B vectors
		Bx = np.zeros(self.N)
		By = np.zeros(self.N)
		Bz = np.zeros(self.N)

		Bx[0] = self.wp[0,1]-self.wp[0,0]-self.xp*self.h
		By[0] = self.wp[1,1]-self.wp[1,0]-self.yp*self.h
		Bz[0] = self.wp[2,1]-self.wp[2,0]-self.zp*self.h

		if self.first:
			A[0,0] = 1
			A[0,1] = 0
			Bx[0] = 0
			By[0] = 0
			Bz[0] = 0

		Bx[1:self.N-1] = self.wp[0,2:self.N] - 2*self.wp[0,1:self.N-1] + self.wp[0,0:self.N-2]
		By[1:self.N-1] = self.wp[1,2:self.N] - 2*self.wp[1,1:self.N-1] + self.wp[1,0:self.N-2]
		Bz[1:self.N-1] = self.wp[2,2:self.N] - 2*self.wp[2,1:self.N-1] + self.wp[2,0:self.N-2]

		self.bx = 3.0/self.h**2*np.dot(np.linalg.inv(A),Bx)
		self.by = 3.0/self.h**2*np.dot(np.linalg.inv(A),By)
		self.bz = 3.0/self.h**2*np.dot(np.linalg.inv(A),Bz)

		self.ax = (self.bx[1:self.N]-self.bx[0:self.N-1])/(3*self.h)
		self.cx = (self.wp[0,1:self.N]-self.wp[0,0:self.N-1])/self.h - self.bx[0:self.N-1]*self.h - self.ax*self.h**2
		self.dx = self.wp[0,0:self.N-1]
		self.bx = self.bx[0:self.N-1]

		self.ay = (self.by[1:self.N]-self.by[0:self.N-1])/(3*self.h)
		self.cy = (self.wp[1,1:self.N]-self.wp[1,0:self.N-1])/self.h - self.by[0:self.N-1]*self.h - self.ay*self.h**2
		self.dy = self.wp[1,0:self.N-1]
		self.by = self.by[0:self.N-1]

		self.az = (self.bz[1:self.N]-self.bz[0:self.N-1])/(3*self.h)
		self.cz = (self.wp[2,1:self.N]-self.wp[2,0:self.N-1])/self.h - self.bz[0:self.N-1]*self.h - self.az*self.h**2
		self.dz = self.wp[2,0:self.N-1]
		self.bz = self.bz[0:self.N-1]

		self.coeff_x =  np.hstack((self.ax[None].T,self.bx[None].T,self.cx[None].T,self.dx[None].T))
		self.coeff_y =  np.hstack((self.ay[None].T,self.by[None].T,self.cy[None].T,self.dy[None].T))
		self.coeff_z =  np.hstack((self.az[None].T,self.bz[None].T,self.cz[None].T,self.dz[None].T))

	def calc_arc_leng(self):
		temp = np.arange(0,self.h+self.dq,self.dq)
		self.s = np.zeros(self.N)
		for i in range(self.N-1):
			x_dq = 3*self.ax[i]*temp**2 + 2*self.bx[i]*temp + self.cx[i]
			y_dq = 3*self.ay[i]*temp**2 + 2*self.by[i]*temp + self.cy[i]
			z_dq = 3*self.az[i]*temp**2 + 2*self.bz[i]*temp + self.cz[i]

			f = np.sqrt(x_dq**2+y_dq**2+z_dq**2)
			self.s[i+1] = self.s[i] + np.sum(f[0:-1] + f[1:])*self.dq/2

			# Create dictionaries 
			self.dict_s[i+1] = self.s[i+1]
		
		self.s = self.s[1:]
		self.L = self.s[-1]

	  
if __name__ == '__main__':
	try:
		rospy.init_node('traj_gen')
		gen_traj()
		rospy.spin()   
	except rospy.ROSInterruptException:
		pass