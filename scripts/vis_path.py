#!/usr/bin/env python
import rospy
import numpy as np
from nav_msgs.msg import Path
from std_msgs.msg import Float64, Float64MultiArray
from geometry_msgs.msg import PoseStamped, TwistStamped

from fla.msg import Coeff

import array_utils as au

class path_vis:
	def __init__(self):
		self.h = 1
		self.dq = 0.01

		# ROS initialization
		self.coeff_sub = rospy.Subscriber('coeffs',Coeff, self.coeffs_CB)
		self.path_pub = rospy.Publisher('splines_path',Path,queue_size=1)

	def coeffs_CB(self,msg):
		self.s = au.multiArray2NumpyArray(msg.s)
		self.coeff_x = au.multiArray2NumpyArray(msg.coeff_x)
		self.coeff_y = au.multiArray2NumpyArray(msg.coeff_y)
		self.coeff_z = au.multiArray2NumpyArray(msg.coeff_z)

		self.N = len(self.s)+1

		rospy.loginfo("Received Coeffs")
		self.calc_path()
		self.send_path()

	def calc_path(self):
		temp = np.arange(0,self.h,self.dq)

		self.ax = self.coeff_x[:,0]
		self.bx = self.coeff_x[:,1]
		self.cx = self.coeff_x[:,2]
		self.dx = self.coeff_x[:,3]

		self.ay = self.coeff_y[:,0]
		self.by = self.coeff_y[:,1]
		self.cy = self.coeff_y[:,2]
		self.dy = self.coeff_y[:,3]

		self.az = self.coeff_z[:,0]
		self.bz = self.coeff_z[:,1]
		self.cz = self.coeff_z[:,2]
		self.dz = self.coeff_z[:,3]

		for i in range(self.N-1):

			x = self.ax[i]*temp**3 + self.bx[i]*temp**2 + self.cx[i]*temp + self.dx[i]
			y = self.ay[i]*temp**3 + self.by[i]*temp**2 + self.cy[i]*temp + self.dy[i]
			z = self.az[i]*temp**3 + self.bz[i]*temp**2 + self.cz[i]*temp + self.dz[i]

			x_dq = 3*self.ax[i]*temp**2 + 2*self.bx[i]*temp + self.cx[i]
			y_dq = 3*self.ay[i]*temp**2 + 2*self.by[i]*temp + self.cy[i]
			z_dq = 3*self.az[i]*temp**2 + 2*self.bz[i]*temp + self.cz[i]

			x_d2q = 6*self.ax[i]*temp + 2*self.bx[i]
			y_d2q = 6*self.ay[i]*temp + 2*self.by[i]
			z_d2q = 6*self.az[i]*temp + 2*self.bz[i]

			x_d3q = 6*self.ax[i]*np.ones(len(x))
			y_d3q = 6*self.ay[i]*np.ones(len(y))
			z_d3q = 6*self.az[i]*np.ones(len(z))

			r = np.hstack((x[None].T,y[None].T,z[None].T))
			dr = np.hstack((x_dq[None].T,y_dq[None].T,z_dq[None].T))
			d2r = np.hstack((x_d2q[None].T,y_d2q[None].T,z_d2q[None].T))
			d3r = np.hstack((x_d3q[None].T,y_d3q[None].T,z_d3q[None].T))

			if i > 0:
				self.r = np.vstack((self.r,r))
				self.dr = np.vstack((self.dr,dr))
				self.d2r = np.vstack((self.d2r,d2r))
				self.d3r = np.vstack((self.d3r,d3r))
			else:
				self.r = r
				self.dr = dr
				self.d2r = d2r
				self.d3r = d3r

	def send_path(self):
		msg = Path()
		msg.header.frame_id = "map"
		for i in range(len(self.r)):
			temp = PoseStamped()
			temp.pose.position.x = self.r[i,0]
			temp.pose.position.y = self.r[i,1]
			temp.pose.position.z = self.r[i,2]
			msg.poses.append(temp)
		self.path_pub.publish(msg)
		

if __name__ == '__main__':
	try:
		rospy.init_node('traj_vis')
		path_vis()
		rospy.spin()   
	except rospy.ROSInterruptException:
		pass