#!/usr/bin/env python
'''
Description: Simple utilities that converts a numpy array to a multiarray and vice versa.  

Created on Dec 12, 2013
Edited on Nov 28 2015

@author: Mark Cutler
@email: markjcutler@gmail.com

@author Brett Lopez
@email btlopez.mit.edu
'''
import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayDimension

#-----------------------------------------------------------------------------
# multiarray to numpy array
#-----------------------------------------------------------------------------
def multiArray2NumpyArray(ma):
    numDim = len(ma.layout.dim)
    s = np.zeros(numDim)
    for i in np.arange(numDim):
        s[i] = ma.layout.dim[i].size
    na = np.reshape(ma.data, tuple(s))
    return na

#-----------------------------------------------------------------------------
# numpy array to multiarray
# TODO: not sure if the stride calculations are correct, although they aren't used
# in the multiArray2NumpyArray function so it might not matter
#-----------------------------------------------------------------------------
def numpyArray2MultiArray(na, ma):
    s = na.shape
    for i in np.arange(len(s)):
        d = MultiArrayDimension()
        d.size = s[i]
        ma.layout.dim.append(d)
    ma.data = np.reshape(na, -1)