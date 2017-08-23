#!/usr/bin/python
###
#
#   Various useful functions.
#     Author: Samuel Bailey <sam@bailey.geek.nz>
#
###

from math import *
import numpy as np


def euclideanDistance2d(x): #euclidean distance from x to (0,0)
    return sqrt((x[0]*x[0]) + (x[1]*x[1]))


def matchingSigns(x, y): #returns true if x and y are both negative or both non-negative
    return (x < 0.0 and y < 0.0) or (x >= 0.0 and y >= 0.0)
    

def rotationToHomogeneous(r): #rotation matrix to homogeneous transformation
    q = np.transpose(np.array([r[0], r[1], r[2], np.zeros(3)], np.float64))
    return np.transpose(np.array([ q[0], q[1], q[2], [ 0.0, 0.0, 0.0, 1.0 ] ], np.float64))


def translationToHomogeneous(t): #translation vector to homogeneous transformation
    tt = np.transpose(np.array([ np.zeros(4), np.zeros(4), np.zeros(4), [ t[0], t[1], t[2], 0.0 ] ], np.float64))
    return tt + np.identity(4)


def matrixToEuler(M):
    heading = atan2(-M[2][0], M[0][0])
    attitude = asin(M[1][0])
    bank = atan2(-M[1][2], M[1][1])
    return heading, attitude, bank


def eulerToMatrix(( heading, attitude, bank )):
    # Convert euler angles back to matrix
    sa, ca = sin(attitude), cos(attitude)
    sb, cb = sin(bank), cos(bank)
    sh, ch = sin(heading), cos(heading)

    return [
        [
            ch*ca,
            (-ch*sa*cb) + (sh*sb),
            (ch*sa*sb) + (sh*cb)
        ],
        [
            sa,
            ca*cb,
            -ca*sb
        ],
        [
            -sh*ca,
            (sh*sa*cb) + (ch*sb),
            (-sh*sa*sb) + (ch*cb)
        ]
    ]


# Given an almost-orthonormal matrix M, returns the closest orthonormal (ie. rotation) matrix
def makeOrthonormal(M):
    return eulerToMatrix(matrixToEuler(M))


def clamp(value, low=0.0, high=1.0):
	return min(max(value, low), high)
