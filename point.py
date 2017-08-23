#!/usr/bin/python
###
#
#   A Point holds a set of (homogeneous or cartesian) coordinates - each defines the position of the point in the appropriate basis.
#     Author: Samuel Bailey <sam@bailey.geek.nz>
#
###

from collections import namedtuple

#we're going to be using points a lot, so a namedtuple should give better performance than a hashmap
def makePointClass():
    pointSchema = [ 'world', 'camera', 'pixel', 'projectedPixel', 'distortedPixel', 'sensor', 'projectedSensor', 'distortedSensor', 'normal', 'projectedNormal', 'distortedNormal' ]
    Point = namedtuple('Point', pointSchema)
    defaultPoint = Point(*map(lambda x: None, range(len(pointSchema))))
    return Point, lambda dict: defaultPoint._replace(**dict)
Point, newPoint = makePointClass()