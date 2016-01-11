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
from splines.msg import State
from acl_fsw.msg import JoyDef

from aclpy import utils
import array_utils as au

import matplotlib.pyplot as pl

controlDT = 1./100

NOT_FLYING = 0
FLYING = 1
LANDING = 2

TAKEOFF = 1
DISABLE = 2
RESET_INTEGRATORS = 4


class master:
	def __init__(self):

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

		self.m = rospy.get_param('mass')

		self.d1 = 0.024/self.m
		self.d2 = 0/self.m

		# ROS initialization
		self.pubOdom = rospy.Publisher('/odom', Float64, queue_size=1)
		self.coeff_sub = rospy.Subscriber('/coeffs',Coeff, self.coeffs_CB)
		self.pubGoal = rospy.Publisher('goal', QuadGoal, queue_size=1)
		self.pubState = rospy.Publisher('/state',State, queue_size=1)
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
		self.s_acc = au.multiArray2NumpyArray(msg.s_a)

		self.coeff_x = au.multiArray2NumpyArray(msg.coeff_x)
		self.coeff_y = au.multiArray2NumpyArray(msg.coeff_y)
		self.coeff_z = au.multiArray2NumpyArray(msg.coeff_z)

		self.coeff_a = au.multiArray2NumpyArray(msg.coeff_a)

		self.v_0 = au.multiArray2NumpyArray(msg.v_0)

		self.N_wp = len(self.s)+1
		self.L = self.s[-1]

		self.s_e = 0

		self.received_coeff = True

		rospy.loginfo("Received Coeffs")


	def send_goal(self):
		self.goal.waypointType = self.wpType
		self.pubGoal.publish(self.goal)

	def send_odometry(self):
		self.pubOdom.publish(self.s_e)

	def send_state(self):
		msg = State()
		msg.v = self.v
		msg.a = self.at
		msg.j = self.jt
		msg.u = self.ut
		self.pubState.publish(msg)

	def eval_splines(self):
		flag = False
		i = 0
		while not flag:
			rospy.loginfo(self.s)
			rospy.loginfo(self.s[i])
			rospy.loginfo(i)
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
				if i > len(self.s):
					flag = True
					rospy.loginfo("Out of bounds")

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


	def calc_vel(self):
		flag = False
		i = 0
		while not flag:
			if self.s_e < self.s_acc[i]:
				if i > 0:
					s_e = self.s_e - self.s_acc[i-1]
				else:
					s_e = self.s_e
				flag = True
				self.v = np.sqrt(2*(1./6*self.coeff_a[i,0]*s_e**6 + 1./5*self.coeff_a[i,1]*s_e**5 + 1./4*self.coeff_a[i,2]*s_e**4 + 1./3*self.coeff_a[i,3]*s_e**3 + 1./2*self.coeff_a[i,4]*s_e**2 + self.coeff_a[i,5]*s_e) + self.v_0[i]**2)
				self.at = self.coeff_a[i,0]*s_e**5 + self.coeff_a[i,1]*s_e**4 + self.coeff_a[i,2]*s_e**3 + self.coeff_a[i,3]*s_e**2 + self.coeff_a[i,4]*s_e + self.coeff_a[i,5]
				self.jt = self.v*(5*self.coeff_a[i,0]*s_e**4 + 4*self.coeff_a[i,1]*s_e**3 + 3*self.coeff_a[i,2]*s_e**2 + 2*self.coeff_a[i,3]*s_e**1 + self.coeff_a[i,4])
				self.ut = self.v*(20*self.coeff_a[i,0]*s_e**3 + 12*self.coeff_a[i,1]*s_e**2 + 6*self.coeff_a[i,2]*s_e + 2*self.coeff_a[i,3])
			else:
				i+=1



	def cmdTimer(self,e):	
		if self.go:
			self.calc_vel()

			self.s_e += self.v*controlDT 

			# rospy.loginfo(self.s_e)

			self.eval_splines()

			if self.v < 0.01:
				self.go = False
				self.received_coeff = False
				self.v = 0
				self.at = 0
				self.jt = 0
				self.ut = 0
				self.s_e = 0
				rospy.loginfo("At last waypoint")

			self.goal.pos.x = self.x
			self.goal.pos.y = self.y
			self.goal.pos.z = self.z
			self.goal.vel.x = self.v*self.T[0]
			self.goal.vel.y = self.v*self.T[1]
			self.goal.vel.z = 0

			aD = self.d1*self.v**2 +self.d2*self.v
			at = self.at + aD			
			an = self.v**2*self.K

			self.dK = self.dK*self.v*(self.N_wp-1)/self.L

			jt = -self.v**3*self.K**2 + self.jt + (2*self.d1*self.v*self.at + self.d2*self.at)
			jn = 2*self.K*self.v*self.at + self.v*self.K*(self.d1*self.v**2+self.d2*self.v) + self.v**2*self.dK

			# Feedforward acceleration and jerk
			self.goal.accel.x = at*self.T[0] + an*self.N[0]  
			self.goal.accel.y = at*self.T[1] + an*self.N[1]
			self.goal.jerk.x = jt*self.T[0] + jn*self.N[0]
			self.goal.jerk.y = jt*self.T[1] + jn*self.N[1]

			self.goal.dyaw = 0*self.v*self.K*np.sin(self.B[2])	

			self.goal.yaw = np.pi/2 #0*np.arctan2(self.T[1],self.T[0])
			

		self.send_goal()
		self.send_odometry()
		self.send_state()


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