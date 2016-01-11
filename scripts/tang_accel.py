#!/usr/bin/env python
import rospy
import numpy as np

class tangAccel:
	def __init__(self):
		rospy.loginfo("Tangential acceleration splines initialized")

	def accel(self,sf,v0,vf,a0,af,j0):
		A = np.matrix([[1,0,0,0,0,0],[1,sf,sf**2,sf**3,sf**4,sf**5],[sf,sf**2/2.,sf**3/3.,sf**4/4.,sf**5/5.,sf**6/6.],[0,1,0,0,0,0],[0,1,2*sf,3*sf**2,4*sf**3,5*sf**4],[0,0,2,6*sf,12*sf**2,20*sf**3]])
		b = np.matrix([[a0],[af],[.5*vf**2-.5*v0**2],[j0],[0],[0]]) # a(0) a(sf) v(sf) j(0) j(sf) u(sf)
		temp = np.linalg.inv(A)*b
		temp = np.asarray(temp.T)[0][::-1]
		self.accelCoeff = temp # Returns in decreasing poly order


	def decel(self,sf,v0,vf,a0,af,j0,u0):
		A = np.matrix([[1,0,0,0,0,0],[1,sf,sf**2,sf**3,sf**4,sf**5],[sf,sf**2/2.,sf**3/3.,sf**4/4.,sf**5/5.,sf**6/6.],[0,1,0,0,0,0],[0,1,2*sf,3*sf**2,4*sf**3,5*sf**4],[0,0,2,0,0,0]])
		b = np.matrix([[a0],[af],[.5*vf**2-.5*v0**2],[j0],[0],[u0]]) # a(0) a(sf) v(sf) j(0) j(sf) u(0)
		temp = np.linalg.inv(A)*b
		temp = np.asarray(temp.T)[0][::-1]
		self.decelCoeff = temp # Returns in decreasing poly order


	