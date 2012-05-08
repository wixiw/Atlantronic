# coding=utf-8
import roslib; roslib.load_manifest('arp_rlu')
import rospy

import sys, os
sys.path.append( "../../../../src/KFLocalizator/PyMockup" )

import random
import json

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import PatchCollection
import matplotlib.patches as mpatches

import LRFSimulator
from BaseClasses import Circle
from BaseClasses import Segment
import LaserOnlyLocalizator


graine = random.randint(0,1000)
random.seed(graine)

xpIndex = 1


#===============================================================================
# Position generation
#===============================================================================

x = random.uniform( -1.3, 1.3)
y = random.uniform( -0.8, 0.8)
h = random.uniform( 0., 2. * np.pi)


rospy.loginfo("true x=%f",x)
rospy.loginfo("true y=%f",y)
rospy.loginfo("true h=%f",h)

tt = np.arange( 0.0, 0.1, 0.1 / 1024.)
xx = np.ones_like(tt) * x
yy = np.ones_like(tt) * y
hh = np.ones_like(tt) * h


lrfsim = LRFSimulator.LRFSimulator()
lrfsim.sigma = 0.01


#===============================================================================
# Scan generation
#===============================================================================

# beacons
radius = 0.04
obj1 = Circle()
obj1.xCenter = 1.560
obj1.yCenter = 0.
obj1.radius = radius
lrfsim.objects.append(obj1)
obj2 = Circle()
obj2.xCenter = -1.555
obj2.yCenter = 1.040
obj2.radius = radius
lrfsim.objects.append(obj2)
obj3 = Circle()
obj3.xCenter = -1.555
obj3.yCenter = -1.040
obj3.radius = radius
lrfsim.objects.append(obj3)


# add random objects on table
nbObjects = 2
for i in range(nbObjects):
  obj = Circle()
  obj.radius  = 0.03
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
  

# add random objects out of the table
nbObjects = 5
for i in range(nbObjects):
  obj = Circle()
  obj.radius  = 0.1
  penetration = True
  while penetration:
    penetration = False
    obj.xCenter = random.uniform( -2.8, 2.8)
    obj.yCenter = random.uniform( -2.0, 2.0)
    if obj.xCenter < 1.7 and  obj.xCenter > -1.7:
        penetration = True
    if obj.yCenter < 1.2 and  obj.yCenter > -1.2:
        penetration = True
    for i in range(len(tt)):
      x_ = xx[i]
      y_ = yy[i]
      penetration = penetration or (np.linalg.norm( np.array( [ [obj.xCenter - x_], [obj.yCenter - y_] ] )) < obj.radius)
  lrfsim.objects.append(obj)
  
# Add segment
sgmt1 = Segment(x= 3.0, y=0., h=0., l=4.4)
lrfsim.objects.append(sgmt1)
sgmt2 = Segment(x=-3.0, y=0., h=np.pi, l=4.4)
lrfsim.objects.append(sgmt2)
sgmt3 = Segment(x= 0., y= 2.2, h= np.pi/2., l=6.0)
lrfsim.objects.append(sgmt3)
sgmt4 = Segment(x= 0., y=-2.2, h=-np.pi/2., l=6.0)
lrfsim.objects.append(sgmt4)
  
  
  
#===============================================================================
# compute scan
#===============================================================================
scan = lrfsim.computeScan(tt, xx, yy, hh)
N = len(scan.theta)


#===============================================================================
# Find position
#===============================================================================
# laserloc = LaserOnlyLocalizator.LaserOnlyLocalizator()
# laserloc.process(scan)


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
         
axe.axis([-3.1, 3.1, -2.3, 2.3])
plt.show()

if not os.path.exists("laseronlyloc_"+ str(xpIndex)):
  os.mkdir("laseronlyloc_"+ str(xpIndex))
scan.export("./laseronlyloc_"+ str(xpIndex) + "/scan.json")

position = {}
position["x"] = x
position["y"] = y
position["h"] = h
if not os.path.exists("laseronlyloc_"+ str(xpIndex)):
  os.mkdir("laseronlyloc_"+ str(xpIndex))
output = open("./laseronlyloc_"+ str(xpIndex) + "/position.json", mode='w')
output.write(json.dumps(position,indent=2,sort_keys=True))
output.close()

