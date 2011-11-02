# coding=utf-8
import sys
sys.path.append( "../../../../src/KFLocalizator/PyMockup" )

import random
import logging
logging.basicConfig(level=logging.INFO)
log = logging.getLogger('main')

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import PatchCollection
import matplotlib.patches as mpatches

import LRFSimulator
from BaseClasses import Circle
from BaseClasses import Segment


#===============================================================================
# trajectory generation
#===============================================================================
ax = random.uniform( -6., 6.)
ay = random.uniform( -6., 6.)
ah = random.uniform( -4. * np.pi, 4. * np.pi)
log.info("ax:%f",ax)
log.info("ay:%f",ay)
log.info("ah:%f",ah*180.0/np.pi)

vx = random.uniform( -3., 3.)
vy = random.uniform( -3., 3.)
vh = random.uniform( -2. * np.pi, 2. * np.pi)
log.info("vx:%f",vx)
log.info("vy:%f",vy)
log.info("vh:%f",vh*180.0/np.pi)

x = random.uniform( -1.3, 1.3)
y = random.uniform( -0.8, 0.8)
h = random.uniform( 0., 2. * np.pi)

tt = np.arange( 0.0, 0.1, 0.1 / 1024.)
xx = tt**2 * ax + tt * vx + x
yy = tt**2 * ay + tt * vy + y
hh = tt**2 * ah + tt * vh + h

xx = np.clip(xx, -1.3, 1.3)
yy = np.clip(yy, -0.8, 0.8)


lrfsim = LRFSimulator.LRFSimulator()
lrfsim.sigma = 0.01


# Add objects
nbObjects = 20
lrfsim.objects = []
for i in range(nbObjects):
  obj = Circle()
  obj.radius  = random.uniform( 0.03, 0.1 )
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
  
# Add border
sgmtN = Segment()
sgmtN.A = np.array([[ 1.5],[ 1.]])
sgmtN.B = np.array([[-1.5],[ 1.]])
lrfsim.objects.append(sgmtN)
sgmtS = Segment()
sgmtS.A = np.array([[-1.5],[-1.]])
sgmtS.B = np.array([[ 1.5],[-1.]])
lrfsim.objects.append(sgmtS)
sgmtE = Segment()
sgmtE.A = np.array([[ 1.5],[-1.]])
sgmtE.B = np.array([[ 1.5],[ 1.]])
lrfsim.objects.append(sgmtE)
sgmtW = Segment()
sgmtW.A = np.array([[-1.5],[ 1.]])
sgmtW.B = np.array([[-1.5],[-1.]])
lrfsim.objects.append(sgmtW)

#===============================================================================
# nbRays = 1000
# angles = arange( 0.0, 2 * pi, 2 * pi / nbRays)
#===============================================================================


# Compute
scan = lrfsim.computeScan(tt, xx, yy, hh)


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
    axe.plot( [xx[-1-i]] , [yy[-1-i]], 'ob' )
    axe.plot( [xx[-1-i], xImpact] , [yy[-1-i], yImpact], '--b' )
    
# borders
N = len(scan.theta)
axe.plot( [xx[-N], xx[-N] + np.cos(hh[-N] + min(scan.theta))], 
         [yy[-N], yy[-N] + np.sin(hh[-N] + np.min(scan.theta))], '-m')
axe.plot( [xx[-1], xx[-1] + np.cos(hh[-1] + max(scan.theta))], 
         [yy[-1], yy[-1] + np.sin(hh[-1] + max(scan.theta))], '-m')


axe.axis([-1.6, 1.6, -1.1, 1.1])
plt.show()

