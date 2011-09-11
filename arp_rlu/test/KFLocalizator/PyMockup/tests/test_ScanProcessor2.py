# coding=utf-8
import sys
sys.path.append( "../../../../src/KFLocalizator/PyMockup" )
import numpy as np
import matplotlib.pyplot as plt
import random

import logging
logging.basicConfig(level=logging.ERROR) 
#logging.basicConfig(level=logging.DEBUG)

import LRFSimulator
import ScanProcessor2
from BaseClasses import Circle
from BaseClasses import Segment
from BaseClasses import pca

lrfsim = LRFSimulator.LRFSimulator()

x = random.uniform( -1.3, 1.3)
y = random.uniform( -0.8, 0.8)
h = random.uniform( 0., 2. * np.pi)
vh = 0. #random.uniform( -2. * np.pi, 2. * np.pi)
print "vh:",vh
tt = np.arange( 0.0, 0.1, 0.1 / 1024.)
xx = np.ones( (len(tt)) ) * x
yy = np.ones( (len(tt)) ) * y
hh = tt * vh + h

lrfsim.sigma = 0.01

  
# Add segment
l = 0.08
lrfsim.objects = []
sgmt1 = Segment(x=1.5, y=0., h=0., l=l)
lrfsim.objects.append(sgmt1)
sgmt2 = Segment(x=-1.5, y=1., h=3*np.pi/4., l=l)
lrfsim.objects.append(sgmt2)
sgmt3 = Segment(x=-1.5, y=-1., h=-3*np.pi/4., l=l)
lrfsim.objects.append(sgmt3)

#sgmt4 = Segment(x=1., y=-1., h=-np.pi/4., l=0.08)
#lrfsim.objects.append(sgmt4)
#sgmt5 = Segment(x=1., y=1., h=np.pi/4, l=0.08)
#lrfsim.objects.append(sgmt5)
#sgmt6 = Segment(x=0., y=1., h=np.pi/2., l=0.08)
#lrfsim.objects.append(sgmt6)


# Compute
scan = lrfsim.computeScan(tt, xx, yy, hh)
L = scan.range.shape[0]

# Process scan
scanproc = ScanProcessor2.ScanProcessor2()
scanproc.clusterParams.maxStddev = 0.2
scanproc.process(scan, tt[-L:], xx[-L:], yy[-L:], hh[-L:])

for o in scanproc.objects:
  print "-----------------"
  print o


# Plot
fig = plt.figure()
ax = fig.add_subplot(111, aspect='equal')
plt.plot( [-1.5, -1.5, 1.5, 1.5, -1.5], [-1., 1., 1., -1., -1.], '-k')
for obj in lrfsim.objects:
  if isinstance(obj, Circle):
    border = np.arange( 0.0, 2 * np.pi, np.pi / 100)
    xBalls = np.cos(border) * obj.radius + obj.xCenter
    yBalls = np.sin(border) * obj.radius + obj.yCenter
    ax.plot( xBalls, yBalls, '-g')

for i in range(len(scan.range)):
  if scan.range[-1-i] > 0.:
    xImpact = xx[-1-i] + np.cos(hh[-1-i] + scan.theta[-1-i]) * scan.range[-1-i]
    yImpact = yy[-1-i] + np.sin(hh[-1-i] + scan.theta[-1-i]) * scan.range[-1-i]
    ax.plot( [xImpact] , [yImpact], 'xb' )
    ax.plot( [xx[i], xImpact] , [yy[i], yImpact], '--b' )
    
N = len(scan.theta)
ax.plot( [xx[-N], xx[-N] + np.cos(hh[-N] + min(scan.theta))], [yy[-N], yy[-N] + np.sin(hh[-N] + np.min(scan.theta))], '-m')
ax.plot( [xx[-1], xx[-1] + np.cos(hh[-1] + max(scan.theta))], 
         [yy[-1], yy[-1] + np.sin(hh[-1] + max(scan.theta))], '-m')

#===============================================================================
# print "min(theta):",(aa[0]         + min(scan.theta))* 180. / pi
# print "max(theta):",(aa[len(scan.theta)-1] + max(scan.theta))* 180. / pi
# print "theta range :", (aa[len(scan.theta)-1] + max(scan.theta) - aa[0] - min(scan.theta)) * 180. / pi
# print "delta angle:", (aa[len(scan.theta)-1] + max(scan.theta) - aa[0] - min(scan.theta))/len(scan.theta) * 180. / pi
#===============================================================================

ax.axis([-1.6, 1.6, -1.1, 1.1])
plt.show()

