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
sigmaInitialHeading = 0.02
sigmaTransOdoVelocity = 0.01
sigmaRotOdoVelocity = 0.001
sigmaLaserRange = 0.01
sigmaLaserAngle = 0.001
kfloc.initialize(0.0,
                 100,
                 initialXPosition, initialYPosition, initialHeading, 
                 sigmaInitialPosition, sigmaInitialHeading,
                 sigmaTransOdoVelocity, sigmaRotOdoVelocity, 
                 sigmaLaserRange, sigmaLaserAngle)

print "======================="
print "Etat initial :"
print "  xPosition:", initialXPosition
print "  yPosition:", initialYPosition
print "  hPosition:", initialHeading
print "  vx:", 0.
print "  vy:", 0.
print "  vz:", 0.


#===============================================================================
# On reste sur place quelques sec
#===============================================================================
odoDurationInSec = 3.
ov = OdoVelocity()
ov.vx = random.normalvariate(0., sigmaTransOdoVelocity)
ov.vy = random.normalvariate(0., sigmaTransOdoVelocity)
ov.vh = random.normalvariate(0., sigmaRotOdoVelocity)
for t in arange(0., odoDurationInSec, 0.01):
  kfloc.newOdoVelocity(t, ov)

#===============================================================================
# Estimee odo avant le scan
#===============================================================================
estim1 = kfloc.getBestEstimate()
print "======================="
print "Estimee avant le scan :"
print "  xPosition:", estim1[1].xRobot
print "  yPosition:", estim1[1].yRobot
print "  hPosition:", estim1[1].hRobot
print "  vx:", estim1[1].velXRobot
print "  vy:", estim1[1].velYRobot
print "  vz:", estim1[1].velHRobot
# print "Covariance avant le scan :"
# print estim1[1].covariance

print "erreur statique apres les odos :"
print "  sur x (en mm):", (estim1[1].xRobot - initialXPosition) * 1000.
print "  sur y (en mm):", (estim1[1].yRobot - initialYPosition) * 1000.
print "  en cap (deg) :", betweenMinusPiAndPlusPi( estim1[1].hRobot - initialHeading ) *180./pi

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

radius = 0.05
obj1 = Object()
obj1.xCenter = 1.5
obj1.yCenter = 0.
obj1.radius = radius
lrfsim.objects.append(obj1)
obj2 = Object()
obj2.xCenter = -1.5
obj2.yCenter = -1.
obj2.radius = radius
lrfsim.objects.append(obj2)
obj3 = Object()
obj3.xCenter = -1.5
obj3.yCenter = 1.
obj3.radius = radius
lrfsim.objects.append(obj3)

#===============================================================================
# obj4 = Object()
# obj4.xCenter = -1.5
# obj4.yCenter = 0.
# obj4.radius = radius
# lrfsim.objects.append(obj4)
# obj5 = Object()
# obj5.xCenter = 1.5
# obj5.yCenter = 1.
# obj5.radius = radius
# lrfsim.objects.append(obj5)
# obj6 = Object()
# obj6.xCenter = 1.5
# obj6.yCenter = -1.
# obj6.radius = radius
# lrfsim.objects.append(obj6)
# 
# obj7 = Object()
# obj7.xCenter = 0.
# obj7.yCenter = 1.
# obj7.radius = radius
# lrfsim.objects.append(obj7)
# obj8 = Object()
# obj8.xCenter = 0.
# obj8.yCenter = -1.
# obj8.radius = radius
# lrfsim.objects.append(obj8)
#===============================================================================

# On calcule enfin le scan
scan = lrfsim.computeScan(xx, yy, hh, tt)


#===============================================================================
# Estimation via le scan
#===============================================================================
kfloc.setBeacons( lrfsim.objects )
kfloc.newScan(odoDurationInSec, scan)

#===============================================================================
# Estimee apres le scan
#===============================================================================
estim2 = kfloc.getBestEstimate()
print "======================="
print "Estimee apres le scan :"
print "  xPosition:", estim2[1].xRobot
print "  yPosition:", estim2[1].yRobot
print "  hPosition:", estim2[1].hRobot
print "  vx:", estim2[1].velXRobot
print "  vy:", estim2[1].velYRobot
print "  vz:", estim2[1].velHRobot
# print "Covariance apres le scan :"
# print estim2[1].covariance

print "Erreur statique apres le scan :"
print "  sur x (en mm):", (estim2[1].xRobot - initialXPosition) * 1000.
print "  sur y (en mm):", (estim2[1].yRobot - initialYPosition) * 1000.
print "  en cap (deg) :", betweenMinusPiAndPlusPi( estim2[1].hRobot - initialHeading ) *180./pi


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


ax.axis([-1.6, 1.6, -1.1, 1.1])
plt.draw()

# arr1 = plt.Arrow(0.1, -0.1, 0.1, 0.1, edgecolor='green')
# ax.add_patch(arr1)

plt.show()
