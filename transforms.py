#!/usr/bin/python
###
#
#   Transformation functions for the pinhole camera model.
#     Author: Samuel Bailey <sam@bailey.geek.nz>
#
###

import math
import numpy as np
from mmath import *

### Transform from world coordinates to pixel coordinates


# 3d world coordinates (in mm) -> 3d camera coordinates (in mm)
def worldToCamera(points, params, yOffset):
    worldToCamera = np.dot(translationToHomogeneous([ params['tx'], params['ty']+yOffset, params['tz'] ]), rotationToHomogeneous(params['rotationMatrix']))

    def transform(point):
        return point._replace(camera = np.dot(worldToCamera, [ point.world[0], point.world[1], point.world[2], 1 ]))
    return map(transform, points)


# 3d camera coordinates (in mm) -> sensor coordinates (2d, in mm)
def projectPoints(points, f):
    cameraToPlane = np.array([ [ f, 0.0, 0.0, 0.0 ], [ 0.0, f, 0.0, 0.0 ], [ 0.0, 0.0, 1.0, 0.0 ] ], np.float64)

    def projectPoint(point):

        # perspective projection of the 3d point onto the plane at z=f
        p = np.dot(cameraToPlane, point.camera)
        p = p / p[2] #perspective division

        # p is now a 2d vector from the center of the image sensor, in mm
        return point._replace(projectedSensor = p)

    return map(projectPoint, points)


# sensor coordinates (2d, in mm) -> normalised image coordinates
def sensorToNormal(points, pixelSize, resolution):
    s2n = np.array([[2.0/(pixelSize[0]*resolution[0]),0.0,0.0],[0.0,2.0/(pixelSize[1]*resolution[1]),0.0],[0.0,0.0,1.0]], np.float64)

    def transform(point):
        p = [ point.projectedSensor[0], point.projectedSensor[1], 1.0 ]
        return point._replace(projectedNormal = np.dot(s2n, p))
    return map(transform, points)


# normalised image coordinates -> distorted normalised image coordinates
def distortPoints(points, kappa):
    def dist(u, d=None, i=8):
        if i == 0:
            d = u
        else:
            d = dist(u, d, i-1)

        rd2 = (d[0]*d[0]) + (d[1]*d[1])
        correction = 1.0 / ( 1.0 - (kappa * rd2) )
        return np.multiply(u, [ correction, correction, 1.0 ])

    def transform(point):
        return point._replace(distortedNormal = dist(point.projectedNormal))
    return map(transform, points)


# (distorted) normalised image coordinates -> pixels
def normalToPixel(points, resolution):
    n2p = np.array([[-resolution[0]/2.0,0.0,resolution[0]/2.0],[0.0,-resolution[1]/2.0,resolution[1]/2.0],[0.0,0.0,1.0]], np.float64)

    def transform(point):
        p = [ point.projectedNormal[0], point.projectedNormal[1], 1.0 ]
        d = [ point.distortedNormal[0], point.distortedNormal[1], 1.0 ]
        return point._replace(projectedPixel = np.dot(n2p, p), distortedPixel = np.dot(n2p, d))
    return map(transform, points)



### Transform from pixel to world coordinates


# pixel coordinates -> normalised image coordinates
def pixelToNormal(points, resolution):
    n2p = np.array([[-resolution[0]/2.0,0.0,resolution[0]/2.0],[0.0,-resolution[1]/2.0,resolution[1]/2.0],[0.0,0.0,1.0]], np.float64)
    p2n = np.linalg.inv(n2p)

    def transform(point):
        p = np.array([ point.pixel[0], point.pixel[1], 1.0 ], np.float64)  #pixel coordinates to homogeneous
        return point._replace(normal = np.dot(p2n, p)) # =  [ xd, yd, 1.0 ]  transform pixel coordinates to sensor coordinates (in mm, origin at center of camera)
    return map(transform, points)


# normalised image coordinates -> undistorted normalised image coordinates
def undistortPoints(points, kappa):
    def transform(point):
        x, y = point.normal[0], point.normal[1]
        correction = 1.0 - ( kappa * ( (x*x) + (y*y) ) )
        return point._replace(normal = np.multiply(point.normal, [ correction, correction, 1.0 ]))
    return map(transform, points)


# normalised image coordinates -> sensor coordinates (2d, in mm)
def normalToSensor(points, resolution, pixelSize):
    s2n = np.array([[2.0/(pixelSize[0]*resolution[0]),0.0,0.0],[0.0,2.0/(pixelSize[1]*resolution[1]),0.0],[0.0,0.0,1.0]], np.float64)
    n2s = np.linalg.inv(s2n)

    def transform(point):
        p = np.array([ point.normal[0], point.normal[1], 1.0 ], np.float64)  #pixel coordinates to homogeneous
        return point._replace(sensor = np.dot(n2s, p)) # =  [ xd, yd, 1.0 ]  transform pixel coordinates to sensor coordinates (in mm, origin at center of camera)
    return map(transform, points)



### Compose a few of the above together to make things easier to read


def pixelToSensor(points, resolution, pixelSize, kappa=0.0):
    return normalToSensor(undistortPoints(pixelToNormal(points, resolution), kappa), resolution, pixelSize)

def sensorToPixel(points, pixelSize, resolution, kappa=0.0):
    return normalToPixel(distortPoints(sensorToNormal(points, pixelSize, resolution), kappa), resolution)

def worldToSensor(points, params, pixelSize, resolution, yOffset, kappa=0.0):
    return sensorToPixel(projectPoints(worldToCamera(points, params, yOffset), params['f']), pixelSize, resolution, kappa)

def worldToPixel(points, params, pixelSize, resolution, yOffset, kappa=0.0):
    return sensorToPixel(projectPoints(worldToCamera(points, params, yOffset), params['f']), pixelSize, resolution, kappa)



# Distance from the origin in camera coordinates to the origin in world coordinates (in mm)
def cameraToWorldDistance(params, yOffset):
    return np.linalg.norm([params['tx'], params['ty']-yOffset, params['tz']])
