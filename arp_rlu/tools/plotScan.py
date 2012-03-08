# coding=utf-8
import sys
sys.path.append( "../src/KFLocalizator/PyMockup" )

import json

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import PatchCollection
import matplotlib.patches as mpatches

import Scan



fileName = "../bin/BeaconDetector__process__scan_0.json"
if len(sys.argv) > 1:
  fileName = sys.argv[1]
  

# importation du scan
scan = Scan.Scan()
scan.load(fileName)

# importation de la trajectoire
tt = scan.tt
xx = scan.xx
yy = scan.yy
hh = scan.hh


#===============================================================================
# Ploting
#===============================================================================
fig = plt.figure()
axe = fig.add_subplot(111, aspect='equal')

# table
plt.plot( [-1.5, -1.5, 1.5, 1.5, -1.5], [-1., 1., 1., -1., -1.], '-k')

## circles
#for obj in lrfsim.objects:
#  if isinstance(obj, Circle):
#    circle = mpatches.Circle(xy=(obj.xCenter, obj.yCenter), radius=obj.radius, ec="green", fc = "none")
#    axe.add_patch(circle)

N = len(scan.theta)
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
axe.plot( [xx[-N], xx[-N] + np.cos(hh[-N] - 2.*np.pi*340./1024.)], 
          [yy[-N], yy[-N] + np.sin(hh[-N] - 2.*np.pi*340./1024.)], '-m')
axe.plot( [xx[-1], xx[-1] + np.cos(hh[-1] + 2.*np.pi*340./1024.)], 
         [yy[-1], yy[-1] + np.sin(hh[-1] + 2.*np.pi*340./1024.)], '-m')

## found objects
#for o in scanproc.objects:
#  axe.plot( [o.xCenter], [o.yCenter], 'or')

         
axe.axis([-1.6, 1.6, -1.1, 1.1])
plt.show()

