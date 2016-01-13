#!/usr/bin/env python
import math
import numpy as np


def wrap(val):
    if val > math.pi:
        val -= 2.0 * math.pi
    if val < -math.pi:
        val += 2.0 * math.pi
    return val


def wrap2Pi(val):
    if val > 2.0 * math.pi:
        val -= 2.0 * math.pi
    if val < 0:
        val += 2.0 * math.pi
    return val


# -----------------------------------------------------------------------------
# wrap a number larger than high back around, starting at zero
# -----------------------------------------------------------------------------
def wrapHightoZero(val, high):
    for i in np.arange(len(val)):
        if val[i] >= high:
            val[i] -= high
    return val


def saturate(val, high, low, sat=[0]):
    sat[0] = 0
    if val > high:
        val = high
        sat[0] = 1
    if val < low:
        val = low
        sat[0] = -1
    return val


def quat2yaw(q):
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                     1 - 2 * (q.y * q.y + q.z * q.z))
    return yaw


def rateLimit(desVal, curVal, minRate, maxRate, dt):
    if desVal > (curVal + maxRate * dt):
        desVal = curVal + maxRate * dt
    elif desVal < (curVal + minRate * dt):
        desVal = curVal + minRate * dt

    return desVal


def sgn(val):
    if val > 0:
        return 1.0
    elif val < 0:
        return -1.0
    else:
        return 0.0


def zeroBand(val, vmin, vmax):
    if (val > vmin and val < vmax):
        val = 0

    return val


def distance2(x1, y1, x2, y2):
    return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)

def distanceN(p1, p2):
    # check inputs
    if len(p1) != len(p2):
        print "Error: input points not equal length"
        return -1

    dist = 0
    for i in range(len(p1)):
        dist += (p1[i] - p2[i])**2

    return math.sqrt(dist)



def norm(x, y):
    return math.sqrt(x * x + y * y)


def rotate2D(x, y, theta):
    if not isinstance(x, np.ndarray):
        L = 1
    else:
        L = len(x)
    R = np.array([[math.cos(theta), -math.sin(theta)],
                 [math.sin(theta), math.cos(theta)]])
    val = np.zeros((2, L))
    val[0, :] = x
    val[1, :] = y
    val_rot = np.dot(R, val)
    x_rot = val_rot[0, :]
    y_rot = val_rot[1, :]
    return x_rot, y_rot


def probVariance(y, yhat):
    diff = np.sum(np.abs(y - yhat))
    return diff / 2.0
