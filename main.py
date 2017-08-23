#!/usr/bin/python
###
#
#   An implementation of Tsai's camera calibration technique.
#     Author: Samuel Bailey <sam@bailey.geek.nz>
#
###


# we need to do some basic python stuff...
from __future__ import print_function
import json
import math
from pprint import pprint

# and some math stuff
import numpy as np
import scipy as sp

# and display some points
import matplotlib
matplotlib.use('agg')
from matplotlib.backends.backend_pdf import PdfPages

verbose = False
printVerbose = print if verbose else lambda *a, **k: None   # http://stackoverflow.com/questions/5980042/how-to-implement-the-verbose-or-v-option-into-a-script



from mmath import *
from point import Point, newPoint
from transforms import *
from calibration import calibrate
from distortion import *
from plot import *



def processStereo(leftCamera, rightCamera):
    worldPoints = map(lambda p: p.world, leftCamera['points'])
    leftParams = leftCamera['params']
    rightParams = rightCamera['params']

    print('\nCamera: %s' % leftCamera['label'])
    print('\n Left:')
    pprint(leftParams)
    

    print('\n Right:')
    pprint(rightParams)
    
    baseline = np.linalg.norm(leftParams['translationVector']-rightParams['translationVector'])

    print('\n baseline:')
    pprint(baseline)

    print('\n distance to camera:')
    camera_midpoint = np.linalg.norm( ( leftParams['translationVector'] + rightParams['translationVector'] ) / 2.0 )
    print(camera_midpoint)

    return {
        'baseline': baseline,
        'points': worldPoints,
    }


def openFile(settings, folder):
    dataFilename = '%s/config.json' % folder

    with open(dataFilename) as dataFile:
        data = json.load(dataFile)

    for n in [ 'pixelSize', 'resolution', 'label' ]:
        settings[n] = data[n]

    def readCsvLine(csvLine):
        values = map(lambda v: float(v), csvLine.split(','))
        return { 'left': newPoint({ 'world': values[:3], 'pixel': values[-4:-2] }), 'right': newPoint({ 'world': values[:3], 'pixel': values[-2:] }) }

    with open('%s/%s' % (folder, data['points'])) as csvFile:
        points = map(readCsvLine, csvFile.readlines())

    leftImage = plt.imread('%s/%s' % (folder, data['images'][0]))
    rightImage = plt.imread('%s/%s' % (folder, data['images'][1]))

    leftPoints, rightPoints = map(lambda o: o['left'], points), map(lambda o: o['right'], points)
    leftCamera = calibrateDistorted(settings, leftPoints, leftImage)
    rightCamera = calibrateDistorted(settings, rightPoints, rightImage)
    world = processStereo(leftCamera, rightCamera)

    return { 'left': leftCamera, 'right': rightCamera, 'world': world }


def openFolders(settings):
    stats = map(lambda folder: openFile(settings, folder), settings['folders'])

    with PdfPages(settings['outputFilename']) as pdf:
        def p(s, l):
            pdf.savefig(plotPoints(s['points'], s['image'], l))
        def f(x):
            p(x['left'], 'Left Sensor')
            p(x['right'], 'Right Sensor')
            pdf.savefig(displayStereo(x))
            pdf.savefig(displayStereoSide(x))
        map(f, stats)
        

def main():
    settings = {
        'camera': 'GoPro Hero 3+ Stereo',
        'yOffset': 0.0,
        'minLowDistortionPoints': 8,
        'maxLowDistortionPoints': 18,
        'numHighDistortionPoints': 8,
        'passes': 8,
        'folders': [ 'data/3d/shoot1' ],
        'outputFilename': 'output.pdf'
    }
    openFolders(settings)

main()
