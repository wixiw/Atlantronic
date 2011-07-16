import sys
sys.path.append( "../../../../src/KFLocalizator/PyMockup" )
from numpy import *
import matplotlib.pyplot as plt
import random

from LRFSimulator import *

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

lrfsim.sigma = 0.01


# Add objects
nbObjects = 20
lrfsim.objects = []
for i in range(nbObjects):
  obj = Object()
  obj.xCenter = random.uniform( -1.5, 1.5)
  obj.yCenter = random.uniform( -1.0, 1.0)
  obj.radius  = random.uniform( 0.03, 0.1 )
  while linalg.norm( array( [ [obj.xCenter - x], [obj.yCenter - y] ] )) < obj.radius:
    obj.xCenter = random.uniform( -1.5, 1.5)
    obj.yCenter = random.uniform( -1.0, 1.0)
  lrfsim.objects.append(obj)

nbRays = 1000
angles = arange( 0.0, 2 * pi, 2 * pi / nbRays)


# Compute
scan = lrfsim.computeScan(xx, yy, aa, tt)


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

print "min(theta):",(aa[0]         + min(scan.theta))* 180. / pi
print "max(theta):",(aa[len(scan.theta)-1] + max(scan.theta))* 180. / pi
print "theta range :", (aa[len(scan.theta)-1] + max(scan.theta) - aa[0] - min(scan.theta)) * 180. / pi
print "delta angle:", (aa[len(scan.theta)-1] + max(scan.theta) - aa[0] - min(scan.theta))/len(scan.theta) * 180. / pi

ax.axis([-1.6, 1.6, -1.1, 1.1])
plt.show()

