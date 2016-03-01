#!/usr/bin/env python
import rospy
import copy
import numpy as np
from nav_msgs.msg import Path
from std_msgs.msg import Float64, Float64MultiArray
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import Joy

from acl_msgs.msg import QuadGoal
from fla.msg import Coeff
from fla.msg import State
from acl_msgs.msg import JoyDef

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

		# ROS initialization
		self.coeff_sub = rospy.Subscriber('/goal_passthrough',QuadGoal, self.goal_CB)
		self.pubGoal = rospy.Publisher('goal', QuadGoal, queue_size=1)
		self.pose_sub  = rospy.Subscriber('pose', PoseStamped, self.pose_CB)
		self.joy = rospy.Subscriber("/joy", Joy, self.joyCB)

		self.pose = PoseStamped()
		self.goal = QuadGoal()
		self.snap_goal = QuadGoal()
		self.joyinfo = JoyDef()

		rospy.Timer(rospy.Duration(controlDT),self.cmdTimer)

		self.alt = 1

		self.x = 0
		self.y = 0
		self.z = 0

		rospy.loginfo("Waiting for coefficients")


	def pose_CB(self, data):
		self.pose = data.pose


	def goal_CB(self,data):

		self.go = True

		self.snap_goal = data

		self.received_coeff = True

		rospy.loginfo("Received Coeffs")


	def send_goal(self):
		self.goal.waypointType = self.wpType
		self.pubGoal.publish(self.goal)
		rospy.loginfo("Sending Goal")
	

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

				# self.goal.yaw = 0 
				# self.dyaw = 0
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
		if self.go and not self.status==LANDING:	

			self.goal.pos = self.snap_goal.pos
			self.goal.vel = self.snap_goal.vel
			self.goal.yaw = self.snap_goal.yaw
			self.goal.dyaw = self.snap_goal.dyaw
			self.goal.accel = self.snap_goal.accel
			self.goal.jerk = self.snap_goal.jerk

		self.send_goal()
		


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