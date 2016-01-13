#!/usr/bin/env python
import rospy
import copy
import numpy as np
from nav_msgs.msg import Path
from std_msgs.msg import Float64, Float64MultiArray
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import Joy

from acl_fsw.msg import QuadGoal
from splines.msg import Coeff
from acl_fsw.msg import JoyDef

import utils
import array_utils as au

import tang_accel as ac


controlDT = 1./100

NOT_FLYING = 0
FLYING = 1
LANDING = 2

TAKEOFF = 1
DISABLE = 2
RESET_INTEGRATORS = 4


class master:
	def __init__(self):

		self.a_max = rospy.get_param('/a_max')

		# Maximum desired velocity
		self.v_max = rospy.get_param('/v_max')

		# Initialize acceleration spline generation
		self.accel_calc = ac.tangAccel()

		self.sf_start = 1.038*(self.v_max**2/self.a_max)
		self.sf_stop = 1.038*(self.v_max**2/self.a_max)

		self.v_kick = 0.05

		self.v_0 = self.v_kick

		self.accel_calc.accel(self.sf_start,self.v_kick,self.v_max,0,0,0)
		self.accel_calc.decel(self.sf_stop,self.v_max,0,0,0,0,0)

		self.startCoeff = self.accel_calc.accelCoeff
		self.stopCoeff = self.accel_calc.decelCoeff

		self.status = NOT_FLYING
		self.transmitting = True
		self.wpType = DISABLE
		self.go = False
		self.t_start = rospy.get_time()

		self.goal = QuadGoal()
		self.goal.waypointType = DISABLE

		self.h = 1
		self.dq = 0.001

		self.received_coeff = False
		self.first_stop = False
		self.stop_now = False

	
		# ROS initialization
		self.pubOdom = rospy.Publisher('/odom', Float64, queue_size=1)
		self.coeff_sub = rospy.Subscriber('/coeffs',Coeff, self.coeffs_CB)
		self.pubGoal = rospy.Publisher('goal', QuadGoal, queue_size=1)
		self.pose_sub  = rospy.Subscriber('pose', PoseStamped, self.pose_CB)
		self.joy = rospy.Subscriber("/joy", Joy, self.joyCB)

		self.pose = PoseStamped()
		self.goal = QuadGoal()
		self.joyinfo = JoyDef()

		rospy.Timer(rospy.Duration(controlDT),self.cmdTimer)

		# Minimum radius of curvature threshold to constraint check on
		self.K_thresh = 0.5

		self.v = 0.0
		self.at = 0.0
		self.jt = 0.0
		self.ut = 0.0

		self.s_e = 0
		self.s_v = 0
		self.L = 0

		self.alt = 0.5

		self.x = 0
		self.y = 0
		self.z = 0

		rospy.loginfo("Waiting for coefficients")


	def pose_CB(self, data):
		self.pose = data.pose

	def coeffs_CB(self,msg):
		self.s = au.multiArray2NumpyArray(msg.s)

		self.coeff_x = au.multiArray2NumpyArray(msg.coeff_x)
		self.coeff_y = au.multiArray2NumpyArray(msg.coeff_y)
		self.coeff_z = au.multiArray2NumpyArray(msg.coeff_z)

		self.N_wp = len(self.s)+1
		self.L = self.s[-1]

		self.s_e = 0

		self.received_coeff = True

		rospy.loginfo("Received Coeffs")

		# self.distance_check()


	def send_goal(self):
		self.goal.waypointType = self.wpType
		self.pubGoal.publish(self.goal)

	def send_odometry(self):
		self.pubOdom.publish(self.s_e)

	def eval_splines(self):
		flag = False
		i = 0
		while not flag:
			if self.s_e < self.s[i]:
				flag = True
				if i > 0:
					q = (self.s_e - self.s[i-1])/(self.s[i]-self.s[i-1])
				else:
					q = self.s_e/self.s[i]
				self.x = self.coeff_x[i,0]*q**3 + self.coeff_x[i,1]*q**2 + self.coeff_x[i,2]*q + self.coeff_x[i,3]
				self.y = self.coeff_y[i,0]*q**3 + self.coeff_y[i,1]*q**2 + self.coeff_y[i,2]*q + self.coeff_y[i,3]
				self.z = self.coeff_z[i,0]*q**3 + self.coeff_z[i,1]*q**2 + self.coeff_z[i,2]*q + self.coeff_z[i,3]
				self.geom_props(i,q)
			else:
				i+=1
				# Protect against out-of-bounds error
				if self.s_e > self.s[-1]:
					flag = True
					i = len(self.s)-1
					self.s_e = self.s[-1]
					q = 1
					self.x = self.coeff_x[i,0]*q**3 + self.coeff_x[i,1]*q**2 + self.coeff_x[i,2]*q + self.coeff_x[i,3]
					self.y = self.coeff_y[i,0]*q**3 + self.coeff_y[i,1]*q**2 + self.coeff_y[i,2]*q + self.coeff_y[i,3]
					self.z = self.coeff_z[i,0]*q**3 + self.coeff_z[i,1]*q**2 + self.coeff_z[i,2]*q + self.coeff_z[i,3]
					self.geom_props(i,q)
				
	def geom_props(self,i,q):

		x_dq = 3*self.coeff_x[i,0]*q**2 + 2*self.coeff_x[i,1]*q + self.coeff_x[i,2]
		y_dq = 3*self.coeff_y[i,0]*q**2 + 2*self.coeff_y[i,1]*q + self.coeff_y[i,2]
		z_dq = 3*self.coeff_z[i,0]*q**2 + 2*self.coeff_z[i,1]*q + self.coeff_z[i,2]

		x_d2q = 6*self.coeff_x[i,0]*q + 2*self.coeff_x[i,1]
		y_d2q = 6*self.coeff_y[i,0]*q + 2*self.coeff_y[i,1]
		z_d2q = 6*self.coeff_z[i,0]*q + 2*self.coeff_z[i,1]

		x_d3q = 6*self.coeff_x[i,0]
		y_d3q = 6*self.coeff_y[i,0]
		z_d3q = 6*self.coeff_z[i,0]

		dr = np.hstack((x_dq,y_dq,z_dq))
		d2r = np.hstack((x_d2q,y_d2q,z_d2q))
		d3r = np.hstack((x_d3q,y_d3q,z_d3q))

		u = np.cross(dr,d2r)
		self.T = dr/np.sqrt(np.sum(dr**2))
		self.K = 0
		self.dK = 0
		self.N = np.zeros(3)
		self.B = np.zeros(3)
	

	
	### WIP: overlap check
	# def distance_check(self):
	# 	# Check if path length is longer than start and stop
	# 		if self.L < (self.sf_start+self.sf_stop):
	# 			self.sf_start = self.L/2
	# 			self.sf_stop = self.L/2
	# 			self.v_max = np.sqrt(self.sf_start*self.a_start/1.038)

	# 			rospy.loginfo("Changing max velocity to: %f",self.v_max)

	# 			self.accel_calc.accel(self.sf_start,self.v,self.v_max,self.at,0,0)
	# 			self.accel_calc.decel(self.sf_stop,self.v_max,0,0,0,0,0)

	# 			self.startCoeff = self.accel_calc.accelCoeff
	# 			self.stopCoeff = self.accel_calc.decelCoeff

	# 			if self.v == 0:
	# 				self.v_0 = self.v_kick
	# 			else:
	# 				self.v_0 = self.v


	def calc_vel(self):

		if self.L-self.s_e <= self.sf_stop:
			self.stop_now = True

		if self.s_v <= self.sf_start and not self.first_stop:
			s_v = self.s_v
			self.v = np.sqrt(2*(1./6*self.startCoeff[0]*s_v**6 + 1./5*self.startCoeff[1]*s_v**5 + 1./4*self.startCoeff[2]*s_v**4 + 1./3*self.startCoeff[3]*s_v**3 + 1./2*self.startCoeff[4]*s_v**2 + self.startCoeff[5]*s_v) + self.v_0**2)
			self.at = self.startCoeff[0]*s_v**5 + self.startCoeff[1]*s_v**4 + self.startCoeff[2]*s_v**3 + self.startCoeff[3]*s_v**2 + self.startCoeff[4]*s_v + self.startCoeff[5]
			self.jt = self.v*(5*self.startCoeff[0]*s_v**4 + 4*self.startCoeff[1]*s_v**3 + 3*self.startCoeff[2]*s_v**2 + 2*self.startCoeff[3]*s_v**1 + self.startCoeff[4])
			self.ut = self.v*(20*self.startCoeff[0]*s_v**3 + 12*self.startCoeff[1]*s_v**2 + 6*self.startCoeff[2]*s_v + 2*self.startCoeff[3])
			self.s_v += self.v*controlDT	
		elif self.stop_now:
			# Check if we need to stop quicker than what we expected
			if self.L < self.sf_stop and not self.first_stop:
				self.sf_stop = self.L
				self.accel_calc.decel(self.sf_stop,self.v,0,0,0,0,0)
				self.stopCoeff = self.accel_calc.decelCoeff

			if not self.first_stop:
				self.s_v = 0
				self.first_stop= True

			s_v = self.s_v
			self.v_0 = self.v_max
			self.v = np.sqrt(2*(1./6*self.stopCoeff[0]*s_v**6 + 1./5*self.stopCoeff[1]*s_v**5 + 1./4*self.stopCoeff[2]*s_v**4 + 1./3*self.stopCoeff[3]*s_v**3 + 1./2*self.stopCoeff[4]*s_v**2 + self.stopCoeff[5]*s_v) + self.v_0**2)
			self.at = self.stopCoeff[0]*s_v**5 + self.stopCoeff[1]*s_v**4 + self.stopCoeff[2]*s_v**3 + self.stopCoeff[3]*s_v**2 + self.stopCoeff[4]*s_v + self.stopCoeff[5]
			self.jt = self.v*(5*self.stopCoeff[0]*s_v**4 + 4*self.stopCoeff[1]*s_v**3 + 3*self.stopCoeff[2]*s_v**2 + 2*self.stopCoeff[3]*s_v**1 + self.stopCoeff[4])
			self.ut = self.v*(20*self.stopCoeff[0]*s_v**3 + 12*self.stopCoeff[1]*s_v**2 + 6*self.stopCoeff[2]*s_v + 2*self.stopCoeff[3])
			self.s_v += self.v*controlDT
		else:
			self.v = self.v_max
			self.at = 0
			self.jt = 0
			self.ut = 0


	def joyCB(self, data):

		if self.status == NOT_FLYING:
			self.goal.yaw = utils.quat2yaw(self.pose.orientation)

		if data.buttons[self.joyinfo.A] and self.status == NOT_FLYING:
			self.status = FLYING
			self.wpType = TAKEOFF

			# set initial goal to current pose
			self.goal.pos = copy.copy(self.pose.position)
			self.goal.vel.x = self.goal.vel.y = self.goal.vel.z = 0
			self.goal.yaw = utils.quat2yaw(self.pose.orientation)
			self.goal.dyaw = 0

			rospy.loginfo("Flying")
				
		
		elif self.go == True and data.buttons[self.joyinfo.Y]:
			rospy.loginfo("Enerting state of hover")
			self.stop_now = True

		# emergency disable
		elif data.buttons[self.joyinfo.B] and self.status == FLYING:
			self.status = NOT_FLYING
			self.wpType = DISABLE
			self.go = False

		# landing
		elif data.buttons[self.joyinfo.X] and self.status == FLYING:
			self.go = False
			self.status = LANDING
			# self.wpType = LAND
			self.goal.pos.x = self.pose.position.x
			self.goal.pos.y = self.pose.position.y
			self.goal.vel.x = 0
			self.goal.vel.y = 0
			self.goal.vel.z = 0 
			self.goal.dyaw = 0
			self.I = 0

		elif data.buttons[self.joyinfo.CENTER] and self.status == FLYING:
			if self.received_coeff:
				self.eval_splines()

				self.goal.pos.x = self.x
				self.goal.pos.y = self.y
				self.goal.pos.z = self.z
				self.goal.vel.x = 0
				self.goal.vel.y = 0
				self.goal.vel.z = 0

				self.goal.yaw = np.pi/2 #0*np.arctan2(self.T[1],self.T[0])
				self.dyaw = 0
				rospy.loginfo("Ready")

			else:
				rospy.loginfo("Haven't received coefficients.")

			
		elif data.buttons[self.joyinfo.START] and self.status == FLYING:
			if self.received_coeff:
				rospy.loginfo("Starting")
				self.go = True
			else:
				rospy.loginfo("Haven't received coefficients.")


		if self.status == TAKEOFF:
			self.goal.pos.z = utils.saturate(self.goal.pos.z+0.0025, self.alt,-0.1)

		if self.status == LANDING:
			if self.pose.position.z > 0.4:
				# fast landing
				self.goal.pos.z = utils.saturate(self.goal.pos.z - 0.0035, 2.0, -0.1)
			else:
				# slow landing
				self.goal.pos.z = utils.saturate(self.goal.pos.z - 0.001, 2.0, -0.1)
			if self.goal.pos.z == -0.1:
				self.status = NOT_FLYING
				self.wpType = DISABLE
				rospy.loginfo("Landed!")



	def cmdTimer(self,e):	
		if self.go:
			self.calc_vel()

			self.s_e += self.v*controlDT 

			# rospy.loginfo(self.s_e)

			self.eval_splines()
			
			if self.v < 0.05:
				self.go = False
				self.received_coeff = False
				self.first_stop = False
				self.stop_now
				self.v = 0
				self.at = 0
				self.jt = 0
				self.ut = 0
				self.s_e = 0
				self.s_v = 0
				rospy.loginfo("At last waypoint")

			self.goal.pos.x = self.x
			self.goal.pos.y = self.y
			self.goal.pos.z = self.z
			self.goal.vel.x = self.v*self.T[0]
			self.goal.vel.y = self.v*self.T[1]
			self.goal.vel.z = 0


			at = self.at 		
			an = self.v**2*self.K

			self.dK = self.dK*self.v*(self.N_wp-1)/self.L

			jt = -self.v**3*self.K**2 + self.jt 
			jn = 2*self.K*self.v*self.at  + self.v**2*self.dK

			# Feedforward acceleration and jerk
			self.goal.accel.x = at*self.T[0] + an*self.N[0]  
			self.goal.accel.y = at*self.T[1] + an*self.N[1]
			self.goal.jerk.x = jt*self.T[0] + jn*self.N[0]
			self.goal.jerk.y = jt*self.T[1] + jn*self.N[1]

			self.goal.dyaw = 0*self.v*self.K*np.sin(self.B[2])	

			self.goal.yaw = np.pi/2 #0*np.arctan2(self.T[1],self.T[0])
			

		self.send_goal()
		self.send_odometry()

if __name__ == '__main__':
	ns = rospy.get_namespace()
	try:
		rospy.init_node('master')
		if str(ns) == '/':
			rospy.logfatal("Need to specify namespace as vehicle name.")
			rospy.logfatal("This is tyipcally accomplished in a launch file.")
			rospy.logfatal("Command line: ROS_NAMESPACE=RQ01 $ rosrun quad_control circle.py")
		else:
			rospy.loginfo("Starting joystick teleop node for: " + ns)
			master()
			rospy.spin()   
	except rospy.ROSInterruptException:
		pass