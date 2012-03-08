# coding=utf-8
import sys
sys.path.append( "../../../../src/KFLocalizator/PyMockup" )

import json

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
import ScanProcessor


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

## on simplifie
#ax = 0.
#ay = 0.
#ah = 0.
#vx = 0.
#vy = 0.
#vh = 0.
#x = 0.
#y = 0.
#h = 0.


tt = np.arange( 0.0, 0.1, 0.1 / 1024.)
xx = tt**2 * ax + tt * vx + x
yy = tt**2 * ay + tt * vy + y
hh = tt**2 * ah + tt * vh + h

xx = np.clip(xx, -1.3, 1.3)
yy = np.clip(yy, -0.8, 0.8)


lrfsim = LRFSimulator.LRFSimulator()
lrfsim.sigma = 0.01


#===============================================================================
# add objects
#===============================================================================
nbObjects = 4
lrfsim.objects = []
for i in range(nbObjects):
  obj = Circle()
  obj.radius  = 0.04
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
  
beacons = []
b1 = Circle()
b1.radius  = 0.04
b1.xCenter = -1.5
b1.yCenter = 1.0
lrfsim.objects.append(b1)
beacons.append(b1)

b2 = Circle()
b2.radius  = 0.04
b2.xCenter = -1.5
b2.yCenter = -1.0
lrfsim.objects.append(b2)
beacons.append(b2)

b3 = Circle()
b3.radius  = 0.04
b3.xCenter = 1.5
b3.yCenter = 0.0
lrfsim.objects.append(b3)
beacons.append(b3)
  
log.info("Nb of objects :%d", len(lrfsim.objects))
for o in lrfsim.objects:
  log.info("x:%f - y:%f - r:%f", o.xCenter, o.yCenter, o.radius)
  
  
#===============================================================================
# compute scan
#===============================================================================
scan = lrfsim.computeScan(tt, xx, yy, hh)
N = len(scan.theta)


#===============================================================================
# Find clusters
#===============================================================================
scanproc = ScanProcessor.ScanProcessor()
scanproc.setScan(scan)
scanproc.findCluster(tt[-N:], xx[-N:], yy[-N:], hh[-N:])

# Beacons
scanproc.beacons = beacons

dictResults = {}
dictResults["nbObjects"] = len(scanproc.objects)
log.info("################################")
log.info("Nb found clusters : %d", len(scanproc.objects))
for i, o in enumerate(scanproc.objects):
  log.info("Object:")
  log.info("  x:%f - y:%f - r:%f", o.xCenter, o.yCenter, o.radius)
  d = {}
  d["xCenter"] = o.xCenter
  d["yCenter"] = o.yCenter
  d["radius"] = o.radius
  dictResults["obj_" + str(i)] = d


xpIndex = 5

#export traj
dictTraj = {}
dictTraj["type"] = "trajectory"
dictTraj["size"] = N
dictTraj["tt"] = list(tt[-N:])
dictTraj["xx"] = list(xx[-N:])
dictTraj["yy"] = list(yy[-N:])
dictTraj["hh"] = list(hh[-N:])
output = open("./traj_"+str(xpIndex)+".json", mode='w')
output.write(json.dumps(dictTraj,indent=2,sort_keys=True))
output.close()

#export scan
scan.export("./scan_"+str(xpIndex)+".json")

# export results
output = open("./results_"+str(xpIndex)+".json", mode='w')
output.write(json.dumps(dictResults,indent=2,sort_keys=True))
output.close()

# export measures
dictMeas = {}
nbMeas = 0
for t in list(tt):
  (xBeacon, yBeacon, rangeMeas, thetaMeas) = scanproc.getBeacons(t)
  if xBeacon is not None:
    dictMeas["meas_"+str(nbMeas)] = {}
    dictMeas["meas_"+str(nbMeas)]["xBeacon"] = xBeacon
    dictMeas["meas_"+str(nbMeas)]["yBeacon"] = yBeacon
    dictMeas["meas_"+str(nbMeas)]["range"] = rangeMeas
    dictMeas["meas_"+str(nbMeas)]["theta"] = thetaMeas
    nbMeas = nbMeas + 1
dictMeas["nbMeas"] = nbMeas   
output = open("./meas_"+str(xpIndex)+".json", mode='w')
output.write(json.dumps(dictMeas,indent=2,sort_keys=True))
output.close()


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
for o in scanproc.objects:
  axe.plot( [o.xCenter], [o.yCenter], 'or')

         
axe.axis([-1.6, 1.6, -1.1, 1.1])
plt.show()

