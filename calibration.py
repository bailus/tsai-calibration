#!/usr/bin/python
###
#
#   Distortion-free Tsai Calibration.
#     Given world (3D position on the cube, in mm) and pixel (2D position in the image, in px) cooordinates as input, along with the pixel size and resolution.
#	  Extracts a set of parameters that describe the camera - see calibrate(points)
#     Author: Samuel Bailey <sam@bailey.geek.nz>
#
###

from __future__ import print_function
import math
from mmath import *


import numpy as np
from numpy.linalg import pinv # Calculate the generalized inverse of a matrix using its singular-value decomposition (SVD) and including all large singular values.
#from scipy.linalg import pinv # Calculate a generalized inverse of a matrix using a least-squares solver.


verbose = False
printVerbose = print if verbose else lambda *a, **k: None   # http://stackoverflow.com/questions/5980042/how-to-implement-the-verbose-or-v-option-into-a-script


def approximateL(points):
    ## Construct the "overdetermined system of linear equations" x = ML
    ## Then "solve" for L

    def constructM(points):
        def constructLineOfM(point):
            d = point.sensor
            w = point.world
            xd, yd = d[0], d[1]
            xw, yw, zw = w[0], w[1], w[2]
            return [ yd*xw, yd*yw, yd*zw, yd, -xd*xw, -xd*yw, -xd*zw ]
        return np.array(map(constructLineOfM, points), np.float64)

    def constructXd(points):
        return np.array(map(lambda point: point.sensor[0], points), np.float64)

    M = constructM(points)
    inverseM = np.linalg.pinv(M)  # M* approximates M inverse
    x = constructXd(points)
    return np.dot(inverseM, x)  # L = M*x


def calculateTy(points, L):
    #find the point that's furthest from the center of the image
    furthestPoint = max(points, key=lambda point: euclideanDistance2d(point.sensor))

    ty = 1.0 / math.sqrt( (L[4]*L[4]) + (L[5]*L[5]) + (L[6]*L[6]) )
    #the above calculation gives the absolute value of Ty

    #to find the correct sign we assume ty is positive...
    r1 = np.array([ L[0], L[1], L[2] ], np.float64) * ty
    r2 = np.array([ L[4], L[5], L[6] ], np.float64) * ty
    tx = L[3] * ty
    xt = np.dot(r1, furthestPoint.world) + tx
    yt = np.dot(r2, furthestPoint.world) + ty

    #check our assumption
    if (matchingSigns(xt, furthestPoint.sensor[0]) and matchingSigns(yt, furthestPoint.sensor[1])):
        return ty
    else:
        return -ty


def calculateSx(L, ty):
    return abs(ty) * math.sqrt( (L[0]*L[0]) + (L[1]*L[1]) + (L[2]*L[2]) )


def calculateRotation(L, ty, sx):
    # The first two rows can be calculated from L, ty and sx
    r1 = np.array([ L[0], L[1], L[2] ], np.float64) * (ty / sx)
    r2 = np.array([ L[4], L[5], L[6] ], np.float64) * ty

    # because r is orthonormal, row 3 can be calculated from row 1 and row 2
    r3 = np.cross(r1, r2)
    return np.hstack((r1, r2, r3)).reshape(3,3)


def calculateTx(L, ty, sx):
    return L[3] * (ty / sx)


def approximateFTz(points, ty, rotationMatrix): # returns [ f, tz ]
    ## Construct system of linear equations x = MP where P = (f,tz)
    uy = np.add(map(lambda point: np.dot(rotationMatrix[1], point.world), points), ty)
    uz = np.array(map(lambda point: np.dot(rotationMatrix[2], point.world), points), np.float64)
    yd = np.array(map(lambda point: point.sensor[1], points), np.float64)
    M = np.column_stack((uy, map(lambda y: -y, yd)))

    ## Find an approximation of P
    inverseM = pinv(M)
    x = uz * yd
    return np.dot(inverseM, x)  # [ f, tz ]


# returns a set of parameters that describe the camera. assumes a distortion-free pinhole model.
def calibrate(points):

    L = approximateL(points)

    ty = calculateTy(points, L)
    printVerbose("ty = %f" % ty)

    sx = calculateSx(L, ty)
    printVerbose("sx = %f" % sx)

    rotationMatrix = makeOrthonormal(calculateRotation(L, ty, sx))
    printVerbose("det(rotationMatrix) = %f" % np.linalg.det(rotationMatrix))

    tx = calculateTx(L, ty, sx)
    printVerbose("tx = %f" % tx)

    [ f, tz ] = approximateFTz(points, ty, rotationMatrix)
    printVerbose("f = %f" % f)
    printVerbose("tz = %f" % tz)

    return { 'f': f, 'tx': tx, 'ty': ty, 'tz': tz, 'rotationMatrix': rotationMatrix }
