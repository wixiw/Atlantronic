# coding=utf-8
import sys
sys.path.append( "../../../../src/KFLocalizator/PyMockup" )

import logging
logging.basicConfig(level=logging.DEBUG)
log = logging.getLogger('main')

import numpy as np
import random
import matplotlib.pyplot as plt
from matplotlib.collections import PatchCollection
import matplotlib.patches as mpatches

from KFLocalizator import *
from LRFSimulator import *

import params_KFLocalizator_Static as params

np.set_printoptions(precision=4)

graine = random.randint(0,1000)
graine_ = random.randint(0,1000)

# divergence
#graine = 66 #graine_ = 562
#graine = 206 #graine_ = 967
#graine = 601 #graine_ = 577   !!!
#graine = 522 #graine_ = 521   !!!
#graine = 913 #graine_ = 339   !!!

# biais
#graine = 431 #graine_ = 118
#graine = 323 #graine_ = 590
#graine = 572 #graine_ = 638
# graine = 155 #graine_ = 828 

# pb de détection de balise 
#graine = 800 #graine_ = 112
#graine = 461 #graine_ = 441




random.seed(graine)
log.info("graine pour la position réelle :%d", graine)
log.info("graine pour la simulation :%d", graine_)


#===============================================================================
# Initialization du KF
#===============================================================================
kfloc = KFLocalizator()
trueX = random.uniform( -1.3, 1.3)
trueY = random.uniform( -0.8, 0.8)
trueH = random.uniform( -2. * np.pi, 2. * np.pi)

random.seed(graine_)

initialXPosition = trueX + random.normalvariate(0., params.simu_cfg["sigmaInitialPosition"])
initialYPosition = trueY + random.normalvariate(0., params.simu_cfg["sigmaInitialPosition"])
initialHeading = trueH + random.normalvariate(0., params.simu_cfg["sigmaInitialHeading"])
kfloc.initialize(0.0,
                 100,
                 initialXPosition, 
                 initialYPosition, 
                 initialHeading, 
                 params.simu_cfg["sigmaInitialPosition"], params.simu_cfg["sigmaInitialHeading"],
                 params.kf_cfg["sigmaTransOdoVelocity"], params.kf_cfg["sigmaRotOdoVelocity"], 
                 params.kf_cfg["sigmaLaserRange"], params.kf_cfg["sigmaLaserAngle"], params.kf_cfg["sigmaSegmentHeading"])
kfloc.Nit = params.kf_cfg["iekf_cfg"]["Nit"]
kfloc.threshold = params.kf_cfg["iekf_cfg"]["threshold"]
kfloc.scanproc.maxDistance = params.kf_cfg["scanproc_cfg"]["maxDistance"]
kfloc.scanproc.thresholdRange = params.kf_cfg["scanproc_cfg"]["thresholdRange"]

kfloc.scanproc.setTrueStaticPositionForDebugOnly(trueX, trueY, trueH)
kfloc.givePerfectLRFMeasures = params.kf_cfg["givePerfectLRFMeasures"]

#===============================================================================
# Simulateur de LRF
#===============================================================================
lrfsim = LRFSimulator()
lrfsim.sigma = params.simu_cfg["sigmaLRF"]


#===============================================================================
# Positionnement des balises
#===============================================================================
radius = 0.05
obj1 = Circle()
obj1.xCenter = 1.5
obj1.yCenter = 0.
obj1.radius = radius
lrfsim.objects.append(obj1)
obj2 = Circle()
obj2.xCenter = -1.5
obj2.yCenter = -1.
obj2.radius = radius
lrfsim.objects.append(obj2)
obj3 = Circle()
obj3.xCenter = -1.5
obj3.yCenter = 1.
obj3.radius = radius
lrfsim.objects.append(obj3)

#l = 0.4
#kfloc.scanproc.clusterParams.maxStddev = l
#lrfsim.objects = []
#sgmt1 = Segment(x=1.5, y=0., h=0., l=l)
#lrfsim.objects.append(sgmt1)
#sgmt2 = Segment(x=-1.5, y=1., h=3*np.pi/4., l=l)
#lrfsim.objects.append(sgmt2)
#sgmt3 = Segment(x=-1.5, y=-1., h=-3*np.pi/4., l=l)
#lrfsim.objects.append(sgmt3)

#===============================================================================
#obj4 = Circle()
#obj4.xCenter = -1.5
#obj4.yCenter = 0.
#obj4.radius = radius
#lrfsim.objects.append(obj4)
#obj5 = Circle()
#obj5.xCenter = 1.5
#obj5.yCenter = 1.
#obj5.radius = radius
#lrfsim.objects.append(obj5)
#obj6 = Circle()
#obj6.xCenter = 1.5
#obj6.yCenter = -1.
#obj6.radius = radius
#lrfsim.objects.append(obj6)
# 
#obj7 = Circle()
#obj7.xCenter = 0.
#obj7.yCenter = 1.
#obj7.radius = radius
#lrfsim.objects.append(obj7)
#obj8 = Circle()
#obj8.xCenter = 0.
#obj8.yCenter = -1.
#obj8.radius = radius
#lrfsim.objects.append(obj8)
#===============================================================================

kfloc.setBeacons( lrfsim.objects )


#===============================================================================
# Affichage initial
#===============================================================================
fig = plt.figure(figsize=(15,10))

ax = fig.add_subplot(111, aspect='equal')
ax.axis([-1.6, 1.6, -1.1, 1.1])

# table
plt.plot( [-1.5, -1.5, 1.5, 1.5, -1.5], [-1., 1., 1., -1., -1.], '-k')

# beacons
for obj in lrfsim.objects:
  if isinstance(obj, Circle):
    border = np.arange( 0.0, 2 * np.pi, np.pi / 100)
    xBalls = np.cos(border) * obj.radius + obj.xCenter
    yBalls = np.sin(border) * obj.radius + obj.yCenter
    ax.plot( xBalls, yBalls, '-g')
  elif isinstance(obj, Segment):
    ax.plot( [obj.A[0,0], obj.B[0,0]], [obj.A[1,0], obj.B[1,0]], '-g')
    ax.plot( [obj.A[0,0], obj.B[0,0]], [obj.A[1,0], obj.B[1,0]], 'dg')

xArrowBeg = initialXPosition
yArrowBeg = initialYPosition
xArrowEnd = 0.1 * np.cos(initialHeading)
yArrowEnd = 0.1 * np.sin(initialHeading)
arrow = plt.Arrow(xArrowBeg, yArrowBeg, xArrowEnd, yArrowEnd, width=0.01, color="red")
ax.add_patch(arrow)
if params.visu_cfg["ellipse"]:
  estim = Estimate()
  estim.xRobot = kfloc.X[0,0]
  estim.yRobot = kfloc.X[1,0]
  estim.hRobot = kfloc.X[2,0]
  estim.covariance = kfloc.P
  xy, width, height, angle = getEllipseParametersFromEstimate(estim)
  ellipse = mpatches.Ellipse(xy, width, height, angle, alpha=0.3, ec="red", fc = "none")
  ax.add_patch(ellipse)

xArrowBeg = trueX
yArrowBeg = trueY
xArrowEnd = 0.1 * np.cos(trueH)
yArrowEnd = 0.1 * np.sin(trueH)
arrow = plt.Arrow(xArrowBeg, yArrowBeg, xArrowEnd, yArrowEnd, width=0.01, color="magenta")
ax.add_patch(arrow)
circ = plt.Circle((trueX, trueY), radius=0.005, ec="magenta", fc="none")
ax.add_patch(circ)
circ = plt.Circle((trueX, trueY), radius=0.025, ec="magenta", fc="none")
ax.add_patch(circ)


plt.draw()

#===============================================================================
# En avant !
#===============================================================================

time = 0.

log.info("=======================")
log.info("Position réelle :")
log.info("  xPosition: %f", trueX)
log.info("  yPosition: %f", trueY)
log.info("  hPosition: %f", trueH)

log.info("=======================")
log.info("Etat initial :")
log.info("  xPosition: %f", initialXPosition)
log.info("  yPosition: %f", initialYPosition)
log.info("  hPosition: %f", initialHeading)
log.info("  vx: %f", 0.)
log.info("  vy: %f", 0.)
log.info("  vz: %f", 0.)

    
log.info("erreur statique sur l'état initial (t = %f) :", time)
log.info("  sur x (en mm): %f", (initialXPosition - trueX) * 1000.)
log.info("  sur y (en mm): %f", (initialYPosition - trueY) * 1000.)
log.info("  en cap (deg) : %f", betweenMinusPiAndPlusPi( initialHeading - trueH ) *180./np.pi)
log.info("covariance : \n%s", repr(kfloc.P))

xOld = initialXPosition
yOld = initialYPosition

time = 0.01

for k in range(params.simu_cfg["Nscans"]):
    log.info("==============================================")
    log.info("==============================================")
    log.info(" TOUR %d", k)
    #===============================================================================
    # On reste sur place quelques sec
    #===============================================================================
    odoDurationInSec = 0.1
    ov = OdoVelocity()
    for t in np.arange(time, time + odoDurationInSec, 0.01):
      ov.vx = random.normalvariate(0., params.simu_cfg["sigmaTransOdoVelocity"])
      ov.vy = random.normalvariate(0., params.simu_cfg["sigmaTransOdoVelocity"])
      ov.vh = random.normalvariate(0., params.simu_cfg["sigmaRotOdoVelocity"])
      kfloc.newOdoVelocity(t, ov)
    time = time + odoDurationInSec
    
    #===============================================================================
    # Estimee odo avant le scan
    #===============================================================================
    estim1 = kfloc.getBestEstimate()
    log.info("=======================")
    #print "Estimee via odo (t =",estim1[0],"): "
    #print "  xPosition:", estim1[1].xRobot
    #print "  yPosition:", estim1[1].yRobot
    #print "  hPosition:", estim1[1].hRobot
    #print "  vx:", estim1[1].velXRobot
    #print "  vy:", estim1[1].velYRobot
    #print "  vz:", estim1[1].velHRobot
    # print "Covariance avant le scan :"
    # print estim1[1].covariance
    
    log.info( "erreur statique apres les odos :")
    log.info( "  sur x (en mm): %f", (estim1[1].xRobot - trueX) * 1000.)
    log.info( "  sur y (en mm): %f", (estim1[1].yRobot - trueY) * 1000.)
    log.info( "  en cap (deg) : %f", betweenMinusPiAndPlusPi( estim1[1].hRobot - trueH ) *180./np.pi)
    log.info("covariance :\n%s", repr(estim1[1].covariance))    
    
    duration = 681. * 0.1 / 1024.
    
    x = trueX
    y = trueY
    h = trueH
    vh = 0.
    tt = np.arange( time - duration, time, 0.1 / 1024.)
    tt = tt[-681:]
    xx = np.ones( (len(tt)) ) * x
    yy = np.ones( (len(tt)) ) * y
    hh = tt * vh + h
    
    
    # On calcule enfin le scan
    scan = lrfsim.computeScan(tt, xx, yy, hh)
    
    
    kfloc.newScan(time, scan)
    
    #===============================================================================
    # Estimee apres le scan
    #===============================================================================
    estims = kfloc.getLastEstimates()
    for estim2 in estims[:-1]:
      log.debug( "-----------------------")
      log.debug( "  Erreur statique post update (t = %f): ", estim2[0])
      log.debug( "    sur x (en mm): %f", (estim2[1].xRobot - trueX) * 1000.)
      log.debug( "    sur y (en mm): %f", (estim2[1].yRobot - trueY) * 1000.)
      log.debug( "    en cap (deg) : %f", betweenMinusPiAndPlusPi( estim2[1].hRobot - trueH ) *180./np.pi)
      log.debug("covariance :\n%s", repr(estim2[1].covariance))

      if params.visu_cfg["intermediary_arrow"]:    
        xArrowBeg = estim2[1].xRobot
        yArrowBeg = estim2[1].yRobot
        xArrowEnd = 0.1 * np.cos(estim2[1].hRobot)
        yArrowEnd = 0.1 * np.sin(estim2[1].hRobot)
        arrow = plt.Arrow(xArrowBeg, yArrowBeg, xArrowEnd, yArrowEnd, width=0.005, alpha=0.3, color="green")
        ax.add_patch(arrow)
        ax.plot( [xOld, xArrowBeg], [yOld, yArrowBeg], '-b' )
        xOld = xArrowBeg
        yOld = yArrowBeg
      
      if params.visu_cfg["intermediary_ellipse"]:
        xy, width, height, angle = getEllipseParametersFromEstimate(estim2[1])
        ellipse = mpatches.Ellipse(xy, width, height, angle, alpha=0.3, ec="green", fc="none")
        ax.add_patch(ellipse)
      
      
      
    estim2 = kfloc.getBestEstimate()
    log.info( "=======================")
    #print "Estimée via scan (t =",estim2[0],"): "
    #print "  xPosition:", estim2[1].xRobot
    #print "  yPosition:", estim2[1].yRobot
    #print "  hPosition:", estim2[1].hRobot
    #print "  vx:", estim2[1].velXRobot
    #print "  vy:", estim2[1].velYRobot
    #print "  vz:", estim2[1].velHRobot
    #print "Covariance apres le scan :"
    #print estim2[1].covariance )
    log.info( "Erreur statique après scan :")
    log.info( "  sur x (en mm): %f", (estim2[1].xRobot - trueX) * 1000.)
    log.info( "  sur y (en mm): %f", (estim2[1].yRobot - trueY) * 1000.)
    log.info( "  en cap (deg) : %f", betweenMinusPiAndPlusPi( estim2[1].hRobot - trueH ) *180./np.pi)
    log.info("covariance :\n%s", repr(estim2[1].covariance))

    xArrowBeg = estim2[1].xRobot
    yArrowBeg = estim2[1].yRobot
    xArrowEnd = 0.1 * np.cos(estim2[1].hRobot)
    yArrowEnd = 0.1 * np.sin(estim2[1].hRobot)
    arrow = plt.Arrow(xArrowBeg, yArrowBeg, xArrowEnd, yArrowEnd, width=0.01, alpha=0.3, color="blue")
    ax.add_patch(arrow)
    
    if params.visu_cfg["ellipse"]:
      xy, width, height, angle = getEllipseParametersFromEstimate(estim2[1])
      ellipse = mpatches.Ellipse(xy, width, height, angle, alpha=0.1, color="blue")
      ax.add_patch(ellipse)
    
    ax.plot( [xOld, xArrowBeg], [yOld, yArrowBeg], '-b' )
    xOld = xArrowBeg
    yOld = yArrowBeg
    
    plt.draw()
    
    # scan
    if k == 0:
      for i in range(len(scan.range)):
        if scan.range[-1-i] > 0.:
          xImpact = trueX + np.cos(trueH + scan.theta[-1-i]) * scan.range[-1-i]
          yImpact = trueY + np.sin(trueH + scan.theta[-1-i]) * scan.range[-1-i]
          ax.plot( [xImpact] , [yImpact], 'xb' )
          ax.plot( [trueX, xImpact] , [trueY, yImpact], '--b' )
      
      # field of view
      ax.plot( [trueX, trueX + np.cos(trueH + min(scan.theta))], [trueY, trueY + np.sin(trueH + min(scan.theta))], '-m')
      ax.plot( [trueX, trueX + np.cos(trueH + max(scan.theta))], [trueY, trueY + np.sin(trueH + max(scan.theta))], '-m')
    

log.info("graine pour la position réelle :%d", graine)
log.info("graine pour la simulation :%d", graine_)

if params.visu_cfg["zoom"]:
  ax.axis([trueX-0.2, trueX+0.2, trueY-0.15, trueY+0.15])
  
  
#plt.title("Iterations : "+ repr(params.simu_cfg["Nscans"]) 
#          + " Erreur sur x="+str((estim2[1].xRobot - trueX) * 1000.) +"mm "
#          + "(+-"+str(1.5 * math.sqrt(estim2[1].covariance[0,0])) +"mm)"
#          + "  sur y="+repr((estim2[1].yRobot - trueY) * 1000.)+"mm "
#          + "(+-"+repr(1.5 * math.sqrt(estim2[1].covariance[1,1]))+"mm)"
#          + " et sur h="+ repr(betweenMinusPiAndPlusPi( estim2[1].hRobot - trueH ) *180./np.pi)+ "deg "
#          + "(+-"+repr(1.5 * math.sqrt(estim2[1].covariance[1,1]) *180./np.pi)+"deg)")
plt.show()
