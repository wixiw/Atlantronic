# coding=utf-8
import sys
sys.path.append( "../../../../src/KFLocalizator/PyMockup" )
import numpy as np
import matplotlib.pyplot as plt
import random

import logging
logging.basicConfig(level=logging.ERROR) #level=logging.DEBUG  

import LRFSimulator
import ScanProcessor2
from BaseClasses import Circle
from BaseClasses import Segment
from BaseClasses import pca

lrfsim = LRFSimulator.LRFSimulator()

x = 0. #random.uniform( -1.5, 1.5)
y = 0. #random.uniform( -1.0, 1.0)
a = 0. #random.uniform( 0., 2. * np.pi)
va = 0. #random.uniform( -2. * np.pi, 2. * np.pi)
print "va:",va
tt = np.arange( 0.0, 0.1, 0.1 / 1024.)
xx = np.ones( (len(tt)) ) * x
yy = np.ones( (len(tt)) ) * y
aa = tt * va + a

lrfsim.sigma = 0.01

  
# Add segment
lrfsim.objects = []
sgmt1 = Segment()
sgmt1.A = np.array([[ 1.5],[ 0.08]])
sgmt1.B = np.array([[ 1.5],[ 0.]])
lrfsim.objects.append(sgmt1)
sgmt2 = Segment()
sgmt2.A = np.array([[ 0. ],[ 1.]])
sgmt2.B = np.array([[ 0.08],[ 1.]])
lrfsim.objects.append(sgmt2)


# Compute
scan = lrfsim.computeScan(xx, yy, aa, tt)

# Process scan
scanproc = ScanProcessor2.ScanProcessor2()
scanproc.clusterParams.maxStddev = 0.5
vPC = scanproc.process(scan, tt[0], xx[0], yy[0], aa[0])

for v in vPC:
  print "-----------------"
  means, stddev, vectors = pca(v.points)
  print "nb points:", v.points.shape[1]
  print "means:"
  print means
  print "stddev:"
  print stddev
  print "vectors:"
  print vectors
  print "ratio:", np.max(stddev) / np.min(stddev)
  print "sgmt direction:"
  print vectors[0:, np.argmax(stddev)]


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
    
N = len(scan.theta)
ax.plot( [xx[-N], xx[-N] + np.cos(aa[-N] + min(scan.theta))], [yy[-N], yy[-N] + np.sin(aa[-N] + np.min(scan.theta))], '-m')
ax.plot( [xx[-1], xx[-1] + np.cos(aa[-1] + max(scan.theta))], 
         [yy[-1], yy[-1] + np.sin(aa[-1] + max(scan.theta))], '-m')

#===============================================================================
# print "min(theta):",(aa[0]         + min(scan.theta))* 180. / pi
# print "max(theta):",(aa[len(scan.theta)-1] + max(scan.theta))* 180. / pi
# print "theta range :", (aa[len(scan.theta)-1] + max(scan.theta) - aa[0] - min(scan.theta)) * 180. / pi
# print "delta angle:", (aa[len(scan.theta)-1] + max(scan.theta) - aa[0] - min(scan.theta))/len(scan.theta) * 180. / pi
#===============================================================================

ax.axis([-1.6, 1.6, -1.1, 1.1])
plt.show()

