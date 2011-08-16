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
tt = arange( 0.0, 0.1, 0.1 / 1024.)
xx = ones( (len(tt)) ) * x
yy = ones( (len(tt)) ) * y
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
N = len(scan.theta)

# Find clusters
scanproc = ScanProcessor()
scanproc.setScan(scan)
scanproc.findCluster(tt[-N:], xx[-N:], yy[-N:], aa[-N:])

# Beacons
scanproc.beacons = lrfsim.objects

print "################################"
print "Nb found clusters :", len(scanproc.objects)
for o in scanproc.objects:
  print "Object:"
  print "  x:",o.xCenter
  print "  y:",o.yCenter
  print "  r:",o.radius
  
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
  if scan.range[-1-i] > 0.:
    xImpact = xx[-1-i] + cos(aa[-1-i] + scan.theta[-1-i]) * scan.range[-1-i]
    yImpact = yy[-1-i] + sin(aa[-1-i] + scan.theta[-1-i]) * scan.range[-1-i]
    ax.plot( [xImpact] , [yImpact], 'xb' )
    ax.plot( [xx[-1-i], xImpact] , [yy[-1-i], yImpact], '--b' )
    
ax.plot( [xx[-N], xx[-N] + cos(aa[-N] + min(scan.theta))], [yy[-N], yy[-N] + sin(aa[-N] + min(scan.theta))], '-m')
ax.plot( [xx[-1], xx[-1] + cos(aa[-1] + max(scan.theta))], 
         [yy[-1], yy[-1] + sin(aa[-1] + max(scan.theta))], '-m')

for o in scanproc.objects:
  ax.plot( [o.xCenter], [o.yCenter], 'or')

         
ax.axis([-1.6, 1.6, -1.1, 1.1])
plt.show()

