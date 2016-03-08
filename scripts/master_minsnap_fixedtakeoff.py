#!/usr/bin/env python
import rospy
import copy
import numpy as np
import rosnode
from nav_msgs.msg import Path
from std_msgs.msg import Float64, Float64MultiArray
from geometry_msgs.msg import PoseStamped, Pose, TwistStamped

from acl_fsw.msg import QuadGoal
from splines.msg import Coeff
from fla_msgs.msg import FlightCommand

import utils
import array_utils as au

controlDT = 1./100

NOT_FLYING = 0
FLYING = 3
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

		self.received_coeff = False
		self.first_stop = False
		self.stop_now = False
	
		# ROS initialization
		self.coeff_sub = rospy.Subscriber('/goal_passthrough',QuadGoal, self.goal_CB)
		self.pubGoal = rospy.Publisher('goal', QuadGoal, queue_size=1)
		self.pose_sub  = rospy.Subscriber('pose', PoseStamped, self.pose_CB)
		self.command = rospy.Subscriber("/flight/command", FlightCommand, self.commandCB)

		self.pose = Pose()
		self.goal = QuadGoal()
		self.snap_goal = QuadGoal()

		rospy.Timer(rospy.Duration(controlDT),self.cmdTimer)


		self.alt = 1.0

		self.x = 0
		self.y = 0
		self.z = 0

		rospy.loginfo("Waiting for commands")


	def pose_CB(self, data):
		self.pose = data.pose

	def goal_CB(self,data):

		self.snap_goal = data
		self.received_coeff = True
		#rospy.loginfo("Received passthrough goal")

		# self.distance_check()


	def send_goal(self):
		self.goal.waypointType = self.wpType
		self.pubGoal.publish(self.goal)


	def commandCB(self, data):

		if self.status == NOT_FLYING:
			self.goal.yaw = utils.quat2yaw(self.pose.orientation)

		if data.command == data.CMD_TAKEOFF and self.status == NOT_FLYING:
			if '/rosbag_ft' in rosnode.get_node_names():
				self.status = TAKEOFF
				self.wpType = TAKEOFF
	
				# set initial goal to current pose
				self.goal.pos = copy.copy(self.pose.position)
				self.goal.vel.x = self.goal.vel.y = self.goal.vel.z = 0
				self.goal.yaw = utils.quat2yaw(self.pose.orientation)
				self.goal.dyaw = 0
	
				rospy.loginfo("Flying")
			else:
				rospy.logerr('Can\'t takeoff without logging !!  --Nick Roy')
				
		
		elif data.command == data.CMD_HOVER and self.go == True:
			rospy.loginfo("Entering state of hover")
			self.stop_now = True

		# emergency disable
		elif data.command == data.CMD_KILL and (self.status == FLYING or self.status == TAKEOFF):
			self.status = NOT_FLYING
			self.wpType = DISABLE
			self.go = False

		# landing
		elif data.command == data.CMD_LAND and (self.status == FLYING or self.status == TAKEOFF):
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

		elif data.command == data.CMD_INIT and self.status == FLYING:
			if self.received_coeff:
				self.goal.pos.x = self.x
				self.goal.pos.y = self.y
				self.goal.pos.z = self.z
				self.goal.vel.x = 0
				self.goal.vel.y = 0
				self.goal.vel.z = 0

				self.goal.yaw = 0  #0*np.arctan2(self.T[1],self.T[0])
				self.dyaw = 0
				rospy.loginfo("Ready")

			else:
				rospy.loginfo("Haven't received coefficients.")

			
		elif data.command == data.CMD_GO and self.status == FLYING:
			if self.received_coeff:
				rospy.loginfo("Starting")
				self.go = True
			else:
				rospy.loginfo("Haven't received coefficients.")



	def cmdTimer(self,e):	
		if self.go and not self.status==LANDING and not self.status==TAKEOFF:	

			rospy.loginfo("Sending passthrough goal")
			self.goal.pos = self.snap_goal.pos
			self.goal.vel = self.snap_goal.vel
			self.goal.yaw = self.snap_goal.yaw
			self.goal.dyaw = self.snap_goal.dyaw
			self.goal.accel = self.snap_goal.accel
			self.goal.jerk = self.snap_goal.jerk

		if self.status == TAKEOFF:
			self.goal.pos.z = utils.saturate(self.goal.pos.z+0.0025, self.alt,-0.1)
			rospy.loginfo("Taking off")
			if abs(self.pose.position.z - self.alt) < 0.05:
				self.status = FLYING
				rospy.loginfo("Takeoff complete: %d", self.status)
		
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
			master()
			rospy.spin()   
	except rospy.ROSInterruptException:
		pass
