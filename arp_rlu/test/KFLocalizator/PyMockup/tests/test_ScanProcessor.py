import sys
sys.path.append( "../../../../src/KFLocalizator/PyMockup" )
from numpy import *
import matplotlib.pyplot as plt
import random

from LRFSimulator import *
from ScanProcessor import *

lrfsim = LRFSimulator()

x = random.uniform( -1.5, 1.5)
y = random.uniform( -1.0, 1.0)
a = random.uniform( 0., 2. * pi)
va = random.uniform( -2. * pi, 2. * pi)
print "va:",va
tt = arange( 0.0, 0.1, 0.0001)
xx = ones( (1000) ) * x
yy = ones( (1000) ) * y
aa = tt * va + a

lrfsim.sigma = 0.003


# Add objects
nbObjects = 5
lrfsim.objects = []
for i in range(nbObjects):
  obj = Object()
  obj.xCenter = random.uniform( -1.5, 1.5)
  obj.yCenter = random.uniform( -1.0, 1.0)
  obj.radius  = 0.04 #random.uniform( 0.01, 0.1 )
  while linalg.norm( array( [ [obj.xCenter - x], [obj.yCenter - y] ] )) < obj.radius:
    obj.xCenter = random.uniform( -1.5, 1.5)
    obj.yCenter = random.uniform( -1.0, 1.0)
  lrfsim.objects.append(obj)
  
print ""
print "Nb of objects :", len(lrfsim.objects)
for o in lrfsim.objects:
  print "x:",o.xCenter
  print "y:",o.yCenter
  print "r:",o.radius
  print ""
  
  
nbRays = 1000
angles = arange( 0.0, 2 * pi, 2 * pi / nbRays)


# Compute
scan = lrfsim.computeScan(xx, yy, aa, tt)


# Find clusters
scanproc = ScanProcessor()
scanproc.setScan(scan)
scanproc.findCluster(xx, yy, aa)

# Beacons
scanproc.beacons = lrfsim.objects

print "################################"
print "Nb found clusters :", len(scanproc.objects)
for o in scanproc.objects:
  print "Object:"
  print "  x:",o.xCenter
  print "  y:",o.yCenter
  print "  r:",o.radius
  b = scanproc.getNearestBeacons(o)
  print "Recognized Beacon:"
  print "  x:",b.xCenter
  print "  y:",b.yCenter
  print "  r:",b.radius
  print "error in mm:", sqrt( (b.xCenter-o.xCenter)**2 + (b.yCenter-o.yCenter)**2 )*1000.
  print "======================"
  
# Plot
fig = plt.figure()
ax = fig.add_subplot(111, aspect='equal')
plt.plot( [-1.5, -1.5, 1.5, 1.5, -1.5], [-1., 1., 1., -1., -1.], '-k')
for obj in lrfsim.objects:
  border = arange( 0.0, 2 * pi, pi / 100)
  xBalls = cos(border) * obj.radius + obj.xCenter
  yBalls = sin(border) * obj.radius + obj.yCenter
  ax.plot( xBalls, yBalls, '-g')

for i in range(len(scan.range)):
  if scan.range[i] > 0.:
    xImpact = xx[i] + cos(aa[i] + scan.theta[i]) * scan.range[i]
    yImpact = yy[i] + sin(aa[i] + scan.theta[i]) * scan.range[i]
    ax.plot( [xImpact] , [yImpact], 'xb' )
    ax.plot( [xx[i], xImpact] , [yy[i], yImpact], '--b' )
    
ax.plot( [xx[0], xx[0] + cos(aa[0] + min(scan.theta))], [yy[0], yy[0] + sin(aa[0] + min(scan.theta))], '-m')
ax.plot( [xx[len(scan.theta)-1], xx[len(scan.theta)-1] + cos(aa[len(scan.theta)-1] + max(scan.theta))], 
         [yy[len(scan.theta)-1], yy[len(scan.theta)-1] + sin(aa[len(scan.theta)-1] + max(scan.theta))], '-m')

for o in scanproc.objects:
  ax.plot( [o.xCenter], [o.yCenter], 'or')

         
ax.axis([-1.6, 1.6, -1.1, 1.1])
plt.show()

