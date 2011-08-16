# coding=utf-8
import sys
sys.path.append( "../../../../src/KFLocalizator/PyMockup" )
from numpy import *
from random import *
import matplotlib.pyplot as plt

from KFLocalizator import *
from LRFSimulator import *

set_printoptions(precision=4)



#===============================================================================
# Initialization triviale du KF
#===============================================================================
kfloc = KFLocalizator()
initialXPosition = random.uniform( -1.5, 1.5)
initialYPosition = random.uniform( -1.0, 1.0)
initialHeading = random.uniform( -2. * pi, 2. * pi)
sigmaInitialPosition = 0.002
sigmaInitialHeading = 0.01
sigmaTransOdoVelocity = 0.01
sigmaRotOdoVelocity = 0.01
sigmaLaserRange = 0.01
sigmaLaserAngle = 0.001
kfloc.initialize(0.0,
                 100,
                 initialXPosition, initialYPosition, initialHeading, 
                 sigmaInitialPosition, sigmaInitialHeading,
                 sigmaTransOdoVelocity, sigmaRotOdoVelocity, 
                 sigmaLaserRange, sigmaLaserAngle)

print "-----------------------"
print "Etat initial :"
print "  xPosition:", initialXPosition
print "  yPosition:", initialYPosition
print "  hPosition:", initialHeading
print "  vx:", 0.
print "  vy:", 0.
print "  vz:", 0.


#===============================================================================
# On reste sur place 1 sec
#===============================================================================
ov = OdoVelocity()
ov.vx = 0.
ov.vy = 0.
ov.vh = 0.
for t in arange(0., 1., 0.01):
  kfloc.newOdoVelocity(t, ov)

#===============================================================================
# Estimee odo avant le scan
#===============================================================================
estim = kfloc.getBestEstimate()
print "-----------------------"
print "Estimee avant le scan :"
print "  xPosition:", estim[1].xRobot
print "  yPosition:", estim[1].yRobot
print "  hPosition:", estim[1].hRobot
print "  vx:", estim[1].velXRobot
print "  vy:", estim[1].velYRobot
print "  vz:", estim[1].velHRobot
# print "Covariance avant le scan :"
# print estim[1].covariance

#===============================================================================
# On simule le scan
#===============================================================================
lrfsim = LRFSimulator()
lrfsim.sigma = 0.01

x = initialXPosition
y = initialYPosition
h = initialHeading
vh = 0.
tt = arange( 0.0, 0.1, 0.1 / 1024.)
xx = ones( (len(tt)) ) * x
yy = ones( (len(tt)) ) * y
hh = tt * vh + h


obj1 = Object()
obj1.xCenter = 1.5
obj1.yCenter = 0.
obj1.radius = 0.05
lrfsim.objects.append(obj1)
obj2 = Object()
obj2.xCenter = -1.5
obj2.yCenter = -1.
obj2.radius = 0.05
lrfsim.objects.append(obj2)
obj3 = Object()
obj3.xCenter = -1.5
obj3.yCenter = 1.
obj3.radius = 0.05
lrfsim.objects.append(obj3)

# On calcule enfin le scan
scan = lrfsim.computeScan(xx, yy, hh, tt)

#===============================================================================
# Affichage
#===============================================================================
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
    xImpact = xx[-1-i] + cos(hh[-1-i] + scan.theta[-1-i]) * scan.range[-1-i]
    yImpact = yy[-1-i] + sin(hh[-1-i] + scan.theta[-1-i]) * scan.range[-1-i]
    ax.plot( [xImpact] , [yImpact], 'xb' )
    ax.plot( [xx[-1-i], xImpact] , [yy[-1-i], yImpact], '--b' )
    
N = len(scan.theta)
ax.plot( [xx[-N], xx[-N] + cos(hh[-N] + min(scan.theta))], [yy[-N], yy[-N] + sin(hh[-N] + min(scan.theta))], '-m')
ax.plot( [xx[-1], xx[-1] + cos(hh[-1] + max(scan.theta))], 
         [yy[-1], yy[-1] + sin(hh[-1] + max(scan.theta))], '-m')

#for o in scanproc.objects:
#  ax.plot( [o.xCenter], [o.yCenter], 'or')

ax.axis([-1.6, 1.6, -1.1, 1.1])
plt.draw()


#===============================================================================
# Estimation via le scan
#===============================================================================
kfloc.setBeacons( lrfsim.objects )
kfloc.newScan(1., scan)

#===============================================================================
# Estimee apres le scan
#===============================================================================
estim = kfloc.getBestEstimate()
print "-----------------------"
print "Estimee apres le scan :"
print "  xPosition:", estim[1].xRobot
print "  yPosition:", estim[1].yRobot
print "  hPosition:", estim[1].hRobot
print "  vx:", estim[1].velXRobot
print "  vy:", estim[1].velYRobot
print "  vz:", estim[1].velHRobot
# print "Covariance apres le scan :"
# print estim[1].covariance




plt.show()
