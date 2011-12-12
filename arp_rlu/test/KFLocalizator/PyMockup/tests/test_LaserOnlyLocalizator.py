# coding=utf-8
import sys
sys.path.append( "../../../../src/KFLocalizator/PyMockup" )

import random
import logging
logging.basicConfig(level=logging.DEBUG)
log = logging.getLogger('main')

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import PatchCollection
import matplotlib.patches as mpatches

import LRFSimulator
from BaseClasses import Circle
from BaseClasses import Segment
import LaserOnlyLocalizator


#===============================================================================
# Position generation
#===============================================================================

x = random.uniform( -1.3, 1.3)
y = random.uniform( -0.8, 0.8)
h = random.uniform( 0., 2. * np.pi)

x = -1.2
y = 0.
h = 0.

log.info("true x=%f",x)
log.info("true y=%f",y)
log.info("true h=%f",h)

tt = np.arange( 0.0, 0.1, 0.1 / 1024.)
xx = np.ones_like(tt) * x
yy = np.ones_like(tt) * y
hh = np.ones_like(tt) * h


lrfsim = LRFSimulator.LRFSimulator()
lrfsim.sigma = 0.01

radius = 0.05
obj1 = Circle()
obj1.xCenter = 1.5
obj1.yCenter = 0.
obj1.radius = radius
lrfsim.objects.append(obj1)
obj2 = Circle()
obj2.xCenter = -1.5
obj2.yCenter = -1.
obj2.radius = radius
lrfsim.objects.append(obj2)
obj3 = Circle()
obj3.xCenter = -1.5
obj3.yCenter = 1.
obj3.radius = radius
lrfsim.objects.append(obj3)

#===============================================================================
# add random objects
#===============================================================================
nbObjects = 2
for i in range(nbObjects):
  obj = Circle()
  obj.radius  = 0.05
  penetration = True
  while penetration:
    penetration = False
    obj.xCenter = random.uniform( -1.5, 1.5)
    obj.yCenter = random.uniform( -1.0, 1.0)
    for i in range(len(tt)):
      x_ = xx[i]
      y_ = yy[i]
      penetration = penetration or (np.linalg.norm( np.array( [ [obj.xCenter - x_], [obj.yCenter - y_] ] )) < obj.radius)
  lrfsim.objects.append(obj)
  
log.info("Nb of objects :%d", len(lrfsim.objects))
for o in lrfsim.objects:
  log.info("x:%f - y:%f - r:%f", o.xCenter, o.yCenter, o.radius)
  
  
#===============================================================================
# compute scan
#===============================================================================
scan = lrfsim.computeScan(tt, xx, yy, hh)
N = len(scan.theta)


#===============================================================================
# Find position
#===============================================================================
laserloc = LaserOnlyLocalizator.LaserOnlyLocalizator()
laserloc.process(scan)


#===============================================================================
# Ploting
#===============================================================================
fig = plt.figure()
axe = fig.add_subplot(111, aspect='equal')

# table
plt.plot( [-1.5, -1.5, 1.5, 1.5, -1.5], [-1., 1., 1., -1., -1.], '-k')

# circles
for obj in lrfsim.objects:
  if isinstance(obj, Circle):
    circle = mpatches.Circle(xy=(obj.xCenter, obj.yCenter), radius=obj.radius, ec="green", fc = "none")
    axe.add_patch(circle)

# scan (ray and impacts)
for i in range(len(scan.range)):
  if scan.range[-1-i] > 0.:
    xImpact = xx[-1-i] + np.cos(hh[-1-i] + scan.theta[-1-i]) * scan.range[-1-i]
    yImpact = yy[-1-i] + np.sin(hh[-1-i] + scan.theta[-1-i]) * scan.range[-1-i]
    axe.plot( [xImpact] , [yImpact], 'xb' )
    axe.plot( [xx[-1-i], xImpact] , [yy[-1-i], yImpact], '--b' )
  axe.plot( [xx[-1-i]] , [yy[-1-i]], 'ob' )
for i in range(0, len(scan.range), 10):
  xArrowBeg = xx[-1-i]
  yArrowBeg = yy[-1-i]
  xArrowEnd = 0.07 * np.cos(hh[-1-i] + scan.theta[-1-i])
  yArrowEnd = 0.07 * np.sin(hh[-1-i] + scan.theta[-1-i])
  arrow = plt.Arrow(xArrowBeg, yArrowBeg, xArrowEnd, yArrowEnd, width=0.005, alpha = 0.1, color="grey")
  axe.add_patch(arrow)
    
# borders
axe.plot( [xx[-N], xx[-N] + np.cos(hh[-N] + min(scan.theta))], 
          [yy[-N], yy[-N] + np.sin(hh[-N] + np.min(scan.theta))], '-m')
axe.plot( [xx[-1], xx[-1] + np.cos(hh[-1] + max(scan.theta))], 
         [yy[-1], yy[-1] + np.sin(hh[-1] + max(scan.theta))], '-m')

# found objects
#for o in scanproc.objects:
#  axe.plot( [o.xCenter], [o.yCenter], 'or')

         
axe.axis([-1.6, 1.6, -1.1, 1.1])
plt.show()

