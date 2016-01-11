#!/usr/bin/env python
import rospy
import numpy as np
from nav_msgs.msg import Path
from std_msgs.msg import Float64, Float64MultiArray
from geometry_msgs.msg import PoseStamped, TwistStamped

# Custom message
from splines.msg import Coeff
from splines.msg import State


import array_utils as au
import tang_accel as ac

class gen_traj:
	def __init__(self):

		self.a_start = rospy.get_param('/a')

		# Maximum desired velocity
		self.v_max = rospy.get_param('/v_max')

		# Initialize acceleration spline generation
		self.accel_calc = ac.tangAccel()


		# Step size for psuedo distance variable
		self.h = 1.0

		# Estimate of arc length
		self.s_e = 0

		# Arc length delta
		self.dq = 0.005

		self.coeff = Coeff()

		# ROS initialization
		self.geom_props = Float64MultiArray()
		self.wp_sub = rospy.Subscriber('/waypoint_list', Path, self.WP_list_CB)
		self.odom_sub = rospy.Subscriber('/odom',Float64,self.odom_CB)
		self.state_sub = rospy.Subscriber('/state',State,self.state_CB)
		self.coeff_pub = rospy.Publisher('/coeffs',Coeff,queue_size=1)
		rospy.loginfo("Trajecotry Spline Initialized")
		rospy.loginfo("Waiting for waypoint list...")

	def odom_CB(self, data):
		self.s_e = data.data

	def state_CB(self,data):
		self.state = data


	def WP_list_CB(self,msg):
		rospy.loginfo("Waypoints received")
		self.data = msg.poses
		now = rospy.get_time()
		self.WP_parse()
		self.line_coeffs()
		self.calc_path()
		self.calc_profiles()
		self.send_coeff()
		then = rospy.get_time()
		print then-now

	def send_coeff(self):
		self.seg_len_MA = Float64MultiArray()
		self.acc_len_MA = Float64MultiArray()

		self.coeff_x_MA = Float64MultiArray()
		self.coeff_y_MA = Float64MultiArray()
		self.coeff_z_MA = Float64MultiArray()

		self.coeff_a_MA = Float64MultiArray()

		self.v_0_MA = Float64MultiArray()

		# Convert numpy array to multi array
		au.numpyArray2MultiArray(self.s,self.seg_len_MA)
		au.numpyArray2MultiArray(self.s_a,self.acc_len_MA)

		au.numpyArray2MultiArray(self.coeff_x,self.coeff_x_MA)
		au.numpyArray2MultiArray(self.coeff_y,self.coeff_y_MA)
		au.numpyArray2MultiArray(self.coeff_z,self.coeff_z_MA)

		au.numpyArray2MultiArray(self.coeff_a,self.coeff_a_MA)

		au.numpyArray2MultiArray(self.v_0,self.v_0_MA)

		self.coeff.s = self.seg_len_MA
		self.coeff.s_a = self.acc_len_MA

		self.coeff.coeff_x = self.coeff_x_MA
		self.coeff.coeff_y = self.coeff_y_MA
		self.coeff.coeff_z = self.coeff_z_MA

		self.coeff.coeff_a = self.coeff_a_MA

		self.coeff.v_0 = self.v_0_MA

		self.coeff_pub.publish(self.coeff)
		
	def WP_parse(self):
		self.N_wp = len(self.data)
		self.wp = [[],[],[]]
		for i in range(self.N_wp):
			self.wp[0].append(self.data[i].pose.position.x)
			self.wp[1].append(self.data[i].pose.position.y)
			self.wp[2].append(self.data[i].pose.position.z)
		self.wp = np.asarray(self.wp)


	def line_coeffs(self):
		# Psuedo distance variable
		self.q = np.arange(0,self.h*(self.N_wp-1),self.h)

		self.cx = self.wp[0,1:] - self.wp[0,0:-1]
		self.cy = self.wp[1,1:] - self.wp[1,0:-1]
		self.cz = self.wp[2,1:] - self.wp[2,0:-1]

		self.dx = self.wp[0,0:self.N_wp-1] 
		self.dy = self.wp[1,0:self.N_wp-1]
		self.dz = self.wp[2,0:self.N_wp-1]

		self.ax = np.zeros(np.size(self.cx))
		self.ay = np.zeros(np.size(self.cy))
		self.az = np.zeros(np.size(self.cz))

		self.bx = np.zeros(np.size(self.cx))
		self.by = np.zeros(np.size(self.cy))
		self.bz = np.zeros(np.size(self.cz))

		self.coeff_x =  np.hstack((self.ax[None].T,self.bx[None].T,self.cx[None].T,self.dx[None].T))
		self.coeff_y =  np.hstack((self.ay[None].T,self.by[None].T,self.cy[None].T,self.dy[None].T))
		self.coeff_z =  np.hstack((self.az[None].T,self.bz[None].T,self.cz[None].T,self.dz[None].T))


	def calc_path(self):
		temp = np.arange(0,self.h,self.dq)
		self.s = np.zeros(self.N_wp)

		for i in range(self.N_wp-1):
			x = self.ax[i]*temp**3 + self.bx[i]*temp**2 + self.cx[i]*temp + self.dx[i]
			y = self.ay[i]*temp**3 + self.by[i]*temp**2 + self.cy[i]*temp + self.dy[i]
			z = self.az[i]*temp**3 + self.bz[i]*temp**2 + self.cz[i]*temp + self.dz[i]

			x_dq = 3*self.ax[i]*temp**2 + 2*self.bx[i]*temp + self.cx[i]
			y_dq = 3*self.ay[i]*temp**2 + 2*self.by[i]*temp + self.cy[i]
			z_dq = 3*self.az[i]*temp**2 + 2*self.bz[i]*temp + self.cz[i]

			x_d2q = 6*self.ax[i]*temp + 2*self.bx[i]
			y_d2q = 6*self.ay[i]*temp + 2*self.by[i]
			z_d2q = 6*self.az[i]*temp + 2*self.bz[i]

			# Calculate arc length 
			f = np.sqrt(x_dq**2+y_dq**2+z_dq**2)
			self.s[i+1] = self.s[i] + np.sum(f[0:-1] + f[1:])*self.dq/2

			r = np.hstack((x[None].T,y[None].T,z[None].T))
			dr = np.hstack((x_dq[None].T,y_dq[None].T,z_dq[None].T))
			d2r = np.hstack((x_d2q[None].T,y_d2q[None].T,z_d2q[None].T))

			if i > 0:
				self.r = np.vstack((self.r,r))
				self.dr = np.vstack((self.dr,dr))
				self.d2r = np.vstack((self.d2r,d2r))
			else:
				self.r = r
				self.dr = dr
				self.d2r = d2r

		self.s = self.s[1:]
		self.L = self.s[-1]

	def calc_profiles(self):

		sf_start = 1.038*(self.v_max**2/self.a_start)
		sf_stop = 1.038*(self.v_max**2/self.a_start)
		sf_c = sf_start

		sf_accel = 1.038*(self.v_max**2 - self.state.v**2)/self.a_start
		sf_decel = 1.038*(self.state.v**2 - 0**2)/self.a_start

		# Initial kick for vehicle
		if self.state.v == 0:
			self.s_a = np.hstack((sf_start, self.L - sf_stop, self.L))

			rospy.loginfo(self.s_a)

			# Check if there's overlap
			temp = self.s_a[1:]-self.s_a[:-1]
			if (temp>0).all():
				self.no_overlap = True
			else:
				self.no_overlap = False
			if self.no_overlap:	
				self.state.v = 0.05
				self.accel_calc.accel(sf_start,self.state.v,self.v_max,0,0,0)
				self.accel_calc.decel(sf_stop,self.v_max,0,0,0,0,0)
				self.startCoeff = self.accel_calc.accelCoeff
				self.stopCoeff = self.accel_calc.decelCoeff
				self.s_a = np.array([sf_accel, self.L - sf_stop, self.L])
				cruise_coeff = np.zeros(6)
				self.coeff_a = np.vstack((self.startCoeff,cruise_coeff,self.stopCoeff))
				self.v_0 = np.array([self.state.v,self.v_max,self.v_max])
			else:
				rospy.loginfo("Overlap, do something here")
				rospy.sleep(10)	
		# Condition for being in cruise phase
		elif sf_accel == 0 and self.L > sf_stop:
			self.accel_calc.decel(sf_stop,self.v_max,0,0,0,0,0)
			self.stopCoeff = self.accel_calc.decelCoeff
			self.s_a = np.array([self.L - sf_stop, self.L])
			cruise_coeff = np.zeros(6)
			self.coeff_a = np.vstack((cruise_coeff,self.stopCoeff))
			self.v_0 = np.array([self.v_max,self.v_max])
		# Condition for stop phase
		elif self.L < sf_stop:
			# if self.L < sf_decel:
			# 	sf_decel = self.L
			# 	rospy.loginfo('Stop sooner')
			self.accel_calc.decel(sf_decel,self.state.v,0,self.state.a,0,self.state.j/self.state.v, self.state.u/self.state.v)
			self.stopCoeff = self.accel_calc.decelCoeff
			rospy.loginfo(self.stopCoeff)
			self.s_a = np.array([self.L])
			self.coeff_a = np.array([self.stopCoeff])
			self.v_0 = np.array([self.state.v])
		# In start-up phase
		else:
			self.accel_calc.accel(sf_start,self.state.v,self.v_max,self.state.a,0,self.state.j/self.state.v)
			self.accel_calc.decel(sf_stop,self.v_max,0,0,0,0,0)
			self.startCoeff = self.accel_calc.accelCoeff
			self.stopCoeff = self.accel_calc.decelCoeff
			self.s_a = np.array([sf_accel, self.L - sf_stop, self.L])
			cruise_coeff = np.zeros(6)
			self.coeff_a = np.vstack((self.startCoeff,cruise_coeff,self.stopCoeff))
			self.v_0 = np.array([self.state.v,self.v_max,self.v_max])
					
			  
if __name__ == '__main__':
	try:
		rospy.init_node('traj_gen')
		gen_traj()
		rospy.spin()   
	except rospy.ROSInterruptException:
		pass