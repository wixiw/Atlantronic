# coding=utf-8
import sys
sys.path.append( "../../../../src/KFLocalizator/PyMockup" )
from numpy import *
from random import *
import matplotlib.pyplot as plt

from KFLocalizator import *
from LRFSimulator import *

set_printoptions(precision=4)

graine = randint(0,1000)
#graine = 128
#graine = 988
graine = 821
random.seed(graine)
print "graine :", graine

#trueX = 0.1
#trueY = -0.1
#trueH = 0.0
#initialXPosition = 1.15624056453
#initialYPosition = -0.655125447019
#initialHeading = 0.22679772196




#===============================================================================
# Initialization triviale du KF
#===============================================================================
kfloc = KFLocalizator()
trueX = random.uniform( -1.3, 1.3)
trueY = random.uniform( -0.8, 0.8)
trueH = random.uniform( -2. * pi, 2. * pi)
sigmaInitialPosition = 0.02 #0.1
sigmaInitialHeading = 0.1
sigmaTransOdoVelocity = 0.001 
sigmaRotOdoVelocity = 0.001
sigmaLaserRange = 0.01
sigmaLaserAngle = 0.001
sigmaSegmentHeading = 0.5
initialXPosition = trueX + random.normalvariate(0., sigmaInitialPosition)
initialYPosition = trueY + random.normalvariate(0., sigmaInitialPosition)
initialHeading = trueH + random.normalvariate(0., sigmaInitialHeading)
kfloc.initialize(0.0,
                 100,
                 initialXPosition, 
                 initialYPosition, 
                 initialHeading, 
                 sigmaInitialPosition, sigmaInitialHeading,
                 sigmaTransOdoVelocity, sigmaRotOdoVelocity, 
                 sigmaLaserRange, sigmaLaserAngle, sigmaSegmentHeading)



#===============================================================================
# Simulateur de LRF
#===============================================================================
lrfsim = LRFSimulator()
lrfsim.sigma = 0.01


#===============================================================================
# Positionnement des balises
#===============================================================================
#radius = 0.05
#obj1 = Circle()
#obj1.xCenter = 1.5
#obj1.yCenter = 0.
#obj1.radius = radius
#lrfsim.objects.append(obj1)
#obj2 = Circle()
#obj2.xCenter = -1.5
#obj2.yCenter = -1.
#obj2.radius = radius
#lrfsim.objects.append(obj2)
#obj3 = Circle()
#obj3.xCenter = -1.5
#obj3.yCenter = 1.
#obj3.radius = radius
#lrfsim.objects.append(obj3)

l = 0.4
kfloc.scanproc.clusterParams.maxStddev = l
lrfsim.objects = []
sgmt1 = Segment(x=1.5, y=0., h=0., l=l)
lrfsim.objects.append(sgmt1)
sgmt2 = Segment(x=-1.5, y=1., h=3*np.pi/4., l=l)
lrfsim.objects.append(sgmt2)
sgmt3 = Segment(x=-1.5, y=-1., h=-3*np.pi/4., l=l)
lrfsim.objects.append(sgmt3)

#===============================================================================
# obj4 = Circle()
# obj4.xCenter = -1.5
# obj4.yCenter = 0.
# obj4.radius = radius
# lrfsim.objects.append(obj4)
# obj5 = Circle()
# obj5.xCenter = 1.5
# obj5.yCenter = 1.
# obj5.radius = radius
# lrfsim.objects.append(obj5)
# obj6 = Circle()
# obj6.xCenter = 1.5
# obj6.yCenter = -1.
# obj6.radius = radius
# lrfsim.objects.append(obj6)
# 
# obj7 = Circle()
# obj7.xCenter = 0.
# obj7.yCenter = 1.
# obj7.radius = radius
# lrfsim.objects.append(obj7)
# obj8 = Circle()
# obj8.xCenter = 0.
# obj8.yCenter = -1.
# obj8.radius = radius
# lrfsim.objects.append(obj8)
#===============================================================================

kfloc.setBeacons( lrfsim.objects )


#===============================================================================
# Affichage initial
#===============================================================================
fig = plt.figure()

ax = fig.add_subplot(111, aspect='equal')
ax.axis([-1.6, 1.6, -1.1, 1.1])

# table
plt.plot( [-1.5, -1.5, 1.5, 1.5, -1.5], [-1., 1., 1., -1., -1.], '-k')

# beacons
for obj in lrfsim.objects:
  if isinstance(obj, Circle):
    border = arange( 0.0, 2 * pi, pi / 100)
    xBalls = cos(border) * obj.radius + obj.xCenter
    yBalls = sin(border) * obj.radius + obj.yCenter
    ax.plot( xBalls, yBalls, '-g')
  elif isinstance(obj, Segment):
    ax.plot( [obj.A[0,0], obj.B[0,0]], [obj.A[1,0], obj.B[1,0]], '-g')
    ax.plot( [obj.A[0,0], obj.B[0,0]], [obj.A[1,0], obj.B[1,0]], 'dg')

xArrowBeg = initialXPosition
yArrowBeg = initialYPosition
xArrowEnd = 0.1 * cos(initialHeading)
yArrowEnd = 0.1 * sin(initialHeading)
arrow = plt.Arrow(xArrowBeg, yArrowBeg, xArrowEnd, yArrowEnd, width=0.01, color="red")
ax.add_patch(arrow)

xArrowBeg = trueX
yArrowBeg = trueY
xArrowEnd = 0.1 * cos(trueH)
yArrowEnd = 0.1 * sin(trueH)
arrow = plt.Arrow(xArrowBeg, yArrowBeg, xArrowEnd, yArrowEnd, width=0.01, color="magenta")
ax.add_patch(arrow)


plt.draw()

#===============================================================================
# En avant !
#===============================================================================

time = 0.

print "======================="
print "Position réelle :"
print "  xPosition:", trueX
print "  yPosition:", trueY
print "  hPosition:", trueH

print "======================="
print "Etat initial :"
print "  xPosition:", initialXPosition
print "  yPosition:", initialYPosition
print "  hPosition:", initialHeading
print "  vx:", 0.
print "  vy:", 0.
print "  vz:", 0.

    
print "erreur statique sur l'état initial (t =",time,") :"
print "  sur x (en mm):", (initialXPosition - trueX) * 1000.
print "  sur y (en mm):", (initialYPosition - trueY) * 1000.
print "  en cap (deg) :", betweenMinusPiAndPlusPi( initialHeading - trueH ) *180./pi

xOld = initialXPosition
yOld = initialYPosition

Ntour = 5
for k in range(Ntour):
    print "=============================================="
    print "=============================================="
    print " TOUR", k
    #===============================================================================
    # On reste sur place quelques sec
    #===============================================================================
    odoDurationInSec = 0.1
    ov = OdoVelocity()
    for t in arange(time, time + odoDurationInSec, 0.01):
      ov.vx = random.normalvariate(0., sigmaTransOdoVelocity)
      ov.vy = random.normalvariate(0., sigmaTransOdoVelocity)
      ov.vh = random.normalvariate(0., sigmaRotOdoVelocity)
      kfloc.newOdoVelocity(t, ov)
    time = time + odoDurationInSec
    
    #===============================================================================
    # Estimee odo avant le scan
    #===============================================================================
    estim1 = kfloc.getBestEstimate()
    print "======================="
    #print "Estimee via odo (t =",estim1[0],"): "
    #print "  xPosition:", estim1[1].xRobot
    #print "  yPosition:", estim1[1].yRobot
    #print "  hPosition:", estim1[1].hRobot
    #print "  vx:", estim1[1].velXRobot
    #print "  vy:", estim1[1].velYRobot
    #print "  vz:", estim1[1].velHRobot
    # print "Covariance avant le scan :"
    # print estim1[1].covariance
    
    print "erreur statique apres les odos :"
    print "  sur x (en mm):", (estim1[1].xRobot - trueX) * 1000.
    print "  sur y (en mm):", (estim1[1].yRobot - trueY) * 1000.
    print "  en cap (deg) :", betweenMinusPiAndPlusPi( estim1[1].hRobot - trueH ) *180./pi
    
    duration = 681. * 0.1 / 1024.
    
    x = trueX
    y = trueY
    h = trueH
    vh = 0.
    tt = arange( time - duration, time, 0.1 / 1024.)
    tt = tt[-681:]
    xx = ones( (len(tt)) ) * x
    yy = ones( (len(tt)) ) * y
    hh = tt * vh + h
    
    
    # On calcule enfin le scan
    scan = lrfsim.computeScan(tt, xx, yy, hh)
    
    
    kfloc.scanproc.setTrueStaticPositionForDebugOnly(trueX, trueY, trueH)
    
    kfloc.newScan(time, scan)
    
    #===============================================================================
    # Estimee apres le scan
    #===============================================================================
    estims = kfloc.getLastEstimates()
    for estim2 in estims[:-1]:
      print "-----------------------"
      print "  Erreur statique post update (t =",estim2[0],"): "
      print "    sur x (en mm):", (estim2[1].xRobot - trueX) * 1000.
      print "    sur y (en mm):", (estim2[1].yRobot - trueY) * 1000.
      print "    en cap (deg) :", betweenMinusPiAndPlusPi( estim2[1].hRobot - trueH ) *180./pi
      
      
      
    estim2 = kfloc.getBestEstimate()
    print "======================="
    #print "Estimée via scan (t =",estim2[0],"): "
    #print "  xPosition:", estim2[1].xRobot
    #print "  yPosition:", estim2[1].yRobot
    #print "  hPosition:", estim2[1].hRobot
    #print "  vx:", estim2[1].velXRobot
    #print "  vy:", estim2[1].velYRobot
    #print "  vz:", estim2[1].velHRobot
    #print "Covariance apres le scan :"
    #print estim2[1].covariance
    print "Erreur statique après scan :"
    print "  sur x (en mm):", (estim2[1].xRobot - trueX) * 1000.
    print "  sur y (en mm):", (estim2[1].yRobot - trueY) * 1000.
    print "  en cap (deg) :", betweenMinusPiAndPlusPi( estim2[1].hRobot - trueH ) *180./pi

    xArrowBeg = estim2[1].xRobot
    yArrowBeg = estim2[1].yRobot
    xArrowEnd = 0.1 * cos(estim2[1].hRobot)
    yArrowEnd = 0.1 * sin(estim2[1].hRobot)
    arrow = plt.Arrow(xArrowBeg, yArrowBeg, xArrowEnd, yArrowEnd, width=0.01, alpha=0.3)
    ax.add_patch(arrow)
    
    ax.plot( [xOld, xArrowBeg], [yOld, yArrowBeg], '-b' )
    xOld = xArrowBeg
    yOld = yArrowBeg
    
    plt.draw()
    
    # scan
    if k == 0:
      for i in range(len(scan.range)):
        if scan.range[-1-i] > 0.:
          xImpact = trueX + cos(trueH + scan.theta[-1-i]) * scan.range[-1-i]
          yImpact = trueY + sin(trueH + scan.theta[-1-i]) * scan.range[-1-i]
          ax.plot( [xImpact] , [yImpact], 'xb' )
          ax.plot( [trueX, xImpact] , [trueY, yImpact], '--b' )
      
      # field of view
      ax.plot( [trueX, trueX + cos(trueH + min(scan.theta))], [trueY, trueY + sin(trueH + min(scan.theta))], '-m')
      ax.plot( [trueX, trueX + cos(trueH + max(scan.theta))], [trueY, trueY + sin(trueH + max(scan.theta))], '-m')
    


#ax.axis([trueX-0.4, trueX+0.4, trueY-0.4, trueY+0.4])
plt.show()
