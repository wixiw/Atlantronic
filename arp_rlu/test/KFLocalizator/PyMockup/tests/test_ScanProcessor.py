# coding=utf-8
import sys
sys.path.append( "../../../../src/KFLocalizator/PyMockup" )
import numpy as np
import matplotlib.pyplot as plt
import random

import LRFSimulator
from BaseClasses import Circle
from BaseClasses import Segment
import ScanProcessor

lrfsim = LRFSimulator.LRFSimulator()

x = random.uniform( -1.5, 1.5)
y = random.uniform( -1.0, 1.0)
a = random.uniform( 0., 2. * np.pi)
va = random.uniform( -2. * np.pi, 2. * np.pi)
print "va:",va
tt = np.arange( 0.0, 0.1, 0.1 / 1024.)
xx = np.ones( (len(tt)) ) * x
yy = np.ones( (len(tt)) ) * y
aa = tt * va + a

lrfsim.sigma = 0.01


# Add objects
nbObjects = 10
lrfsim.objects = []
for i in range(nbObjects):
  obj = Circle()
  obj.xCenter = random.uniform( -1.5, 1.5)
  obj.yCenter = random.uniform( -1.0, 1.0)
  obj.radius  = 0.04
  while np.linalg.norm( np.array( [ [obj.xCenter - x], [obj.yCenter - y] ] )) < obj.radius:
    obj.xCenter = random.uniform( -1.5, 1.5)
    obj.yCenter = random.uniform( -1.0, 1.0)
  lrfsim.objects.append(obj)
  
print ""
print "Nb of objects :", len(lrfsim.objects)
for o in lrfsim.objects:
  print "x:", o.xCenter, " - y:", o.yCenter, " - r:", o.radius
  
  
# Compute
scan = lrfsim.computeScan(tt, xx, yy, aa)
N = len(scan.theta)


# Find clusters
scanproc = ScanProcessor.ScanProcessor()
scanproc.setScan(scan)
scanproc.findCluster(tt[-N:], xx[-N:], yy[-N:], aa[-N:])

# Beacons
scanproc.beacons = lrfsim.objects

print "################################"
print "Nb found clusters :", len(scanproc.objects)
for o in scanproc.objects:
  print "Object:"
  print "x:", o.xCenter, " - y:", o.yCenter, " - r:", o.radius
#  

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
    xImpact = xx[-1-i] + np.cos(aa[-1-i] + scan.theta[-1-i]) * scan.range[-1-i]
    yImpact = yy[-1-i] + np.sin(aa[-1-i] + scan.theta[-1-i]) * scan.range[-1-i]
    ax.plot( [xImpact] , [yImpact], 'xb' )
    ax.plot( [xx[i], xImpact] , [yy[i], yImpact], '--b' )
    
ax.plot( [xx[-N], xx[-N] + np.cos(aa[-N] + min(scan.theta))], [yy[-N], yy[-N] + np.sin(aa[-N] + np.min(scan.theta))], '-m')
ax.plot( [xx[-1], xx[-1] + np.cos(aa[-1] + max(scan.theta))], 
         [yy[-1], yy[-1] + np.sin(aa[-1] + max(scan.theta))], '-m')

#for o in scanproc.objects:
#  ax.plot( [o.xCenter], [o.yCenter], 'or')

         
ax.axis([-1.6, 1.6, -1.1, 1.1])
plt.show()

