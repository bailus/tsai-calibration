#!/usr/bin/python
###
#
#   Functions to plot the results of calibration.
#     Author: Samuel Bailey <sam@bailey.geek.nz>
#
###

import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from matplotlib.pyplot import cm
from mpl_toolkits.mplot3d import Axes3D


def plotPoints(points, image, title=''):
    fig, (ax) = plt.subplots(1)
    fig.set_size_inches(6,6)

    markersize=6
    ax.imshow(image, alpha=0.4)

    x, y = zip(*map(lambda point: point.pixel, points))
    ax.plot(x, y, 'rx', markersize=markersize/math.sqrt(2), label='actual')
    
    px, py = zip(*map(lambda point: point.projectedPixel[:2], points))
    ax.plot(px, py, 'k+', markersize=markersize, label='projected')
    
    ux, uy = zip(*map(lambda point: point.distortedPixel[:2], points))
    ax.plot(ux, uy, 'k+', markersize=markersize*2, label='distorted')
    
    ax.set_title('%s' % title)
    ax.set_xlabel('x (px)')
    ax.set_ylabel('y (px)')
    ax.legend()

    return fig


def plotStats(errorss, kappass, resolutions, labels, title=None):
    n = len(errorss)
    fig, (ax1, ax2) = plt.subplots(2, sharex=True)
    fig.set_size_inches(6,6)

    if (title):
        ax1.set_title(title)
    ax1.set_ylabel('K1')

    ax2.set_ylabel('Mean error (pixels)')

    ax2.set_xlabel('Iteration (t)')
    xaxis = range(-1,len(errorss[0])-1)

    colors = cm.Dark2(np.linspace(0,1,n))

    for i in range(n):
        kappas = kappass[i]
        errors = errorss[i]
        color = colors[i]
        resolution = resolutions[i]
        label = labels[i]

        ax1.plot(xaxis, kappas, marker='.', c=color, label=label)

        #ax2.plot(map(lambda e: e.minmax[1], errors), ':', c=color)
        ax2.plot(xaxis, map(lambda e: e.mean, errors), marker='.', c=color, label=label)
        #ax2.plot(map(lambda e: e.minmax[0], errors), ':', c=color)

    ax2.legend()

    return fig


def displayStereo(model):
    world = model['world']
    points = world['points']
    fig = plt.figure()
    ax = fig.add_subplot(111)
    fig.set_size_inches(6,6)

    markersize=6

    def displayCamera(c, label, color):
        params = c['params']
        invRT = np.linalg.inv(params['RT'])
        p = np.dot(invRT, [ 0.0, 0.0, 0.0, 1.0 ])
        px = np.dot(invRT, [ -10.0, 0.0, 0.0, 1.0 ])
        pz = np.dot(invRT, [ 0.0, 0.0, -10.0, 1.0 ])
        ax.plot([ p[0] ], [ p[2] ], 'k.', markersize=markersize, color=color, label=label)
        dx = px-p
        dz = pz-p
        ax.arrow(p[0]-dx[0], p[2]-dx[2], dx[0]*2.0, dx[2]*2.0, head_width=markersize*0.3, head_length=markersize*0.4, fc=color, ec=color, label=label, linestyle=':')
        ax.arrow(p[0]-dz[0], p[2]-dz[2], dz[0]*2.0, dz[2]*2.0, head_width=markersize*0.3, head_length=markersize*0.4, fc=color, ec=color, label=label, linestyle='-')

    displayCamera(model['left'], 'Left Camera', 'g')
    displayCamera(model['right'], 'Right Camera', 'b')


    x = map(lambda point: point[0], points)
    z = map(lambda point: point[2], points)
    ax.plot(x, z, 'r+', markersize=markersize, label='Calibration point')

    ax.set_xlabel('x (mm)')
    ax.set_ylabel('z (mm)')

    ax.set_title('Cameras and Calibration Points\nTop-Down View (In world coordinates)')
    ax.legend()
    ax.grid()
    return fig


def displayStereoSide(model):
    world = model['world']
    points = world['points']
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    fig.set_size_inches(6,6)
    ax.view_init(30, 45)

    markersize=6

    def displayCamera(c, label, color):
        params = c['params']
        invRT = np.linalg.inv(params['RT'])
        p = np.dot(invRT, [ 0.0, 0.0, 0.0, 1.0 ])
        px = np.dot(invRT, [ 10.0, 0.0, 0.0, 1.0 ])
        py = np.dot(invRT, [ 0.0, 10.0, 0.0, 1.0 ])
        pz = np.dot(invRT, [ 0.0, 0.0, 10.0, 1.0 ])
        ax.plot([ p[0] ], [ p[1] ], 'k.', zs=[ p[2] ], markersize=markersize, color=color, label=label)
        dx = px-p
        dy = py-p
        dz = pz-p
        ax.quiver(p[0], p[1], p[2], dx[0], dx[1], dx[2], color=color)
        ax.quiver(p[0], p[1], p[2], dy[0], dy[1], dy[2], color=color)
        ax.quiver(p[0], p[1], p[2], dz[0], dz[1], dz[2], color=color)

    displayCamera(model['left'], 'Left Camera', 'g')
    displayCamera(model['right'], 'Right Camera', 'b')


    x = map(lambda point: point[0], points)
    y = map(lambda point: point[1], points)
    z = map(lambda point: point[2], points)
    ax.plot(x, y, 'r+', zs=z, markersize=markersize, label='Calibration point')

    ax.set_xlabel('x (mm)')
    ax.set_ylabel('y (mm)')
    ax.set_zlabel('z (mm)')

    ax.grid()
    ax.set_title('Cameras and Calibration Points\nOrthographic Projection (In world coordinates)')
    ax.legend()
    return fig
