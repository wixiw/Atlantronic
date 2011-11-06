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

import KFLocalizator
import LRFSimulator

from BaseClasses import *

import params_KFLocalizator_Dynamic as params

np.set_printoptions(precision=4)

graine = random.randint(0,1000)
graine_ = random.randint(0,1000)

graine = 528
graine_ = 589

log.info("graine pour la position réelle :%d", graine)
log.info("graine pour la simulation :%d", graine_)


#===============================================================================
# Generation de la trajectoire
#===============================================================================
import GenerateMouvement
random.seed(graine)
tt,xx,yy,hh,vx,vy,vh,ax,ay,ah = GenerateMouvement.gen(params.simu_cfg)


#===============================================================================
# Initialization du KF
#===============================================================================
kfloc = KFLocalizator.KFLocalizator()
random.seed(graine_)

initialXPosition = random.normalvariate(xx[0], params.simu_cfg["sigmaInitialPosition"])
initialYPosition = random.normalvariate(yy[0], params.simu_cfg["sigmaInitialPosition"])
initialHeading   = random.normalvariate(hh[0], params.simu_cfg["sigmaInitialHeading"])
kfloc.initialize(tt[0],
                 100,
                 initialXPosition, 
                 initialYPosition, 
                 initialHeading, 
                 params.kf_cfg["sigmaInitialPosition"], params.kf_cfg["sigmaInitialHeading"],
                 params.kf_cfg["sigmaTransOdoVelocity"], params.kf_cfg["sigmaRotOdoVelocity"], 
                 params.kf_cfg["sigmaLaserRange"], params.kf_cfg["sigmaLaserAngle"], params.kf_cfg["sigmaSegmentHeading"])
kfloc.Nit = params.kf_cfg["iekf_cfg"]["Nit"]
kfloc.threshold = params.kf_cfg["iekf_cfg"]["threshold"]
kfloc.scanproc.maxDistance = params.kf_cfg["scanproc_cfg"]["maxDistance"]
kfloc.scanproc.thresholdRange = params.kf_cfg["scanproc_cfg"]["thresholdRange"]

kfloc.givePerfectLRFMeasures = params.kf_cfg["givePerfectLRFMeasures"]
kfloc.scanproc.setTrueStaticPositionForDebugOnly(xx[0], yy[0], hh[0])

estim = kfloc.getBestEstimate()
log.info( "**********************************************")
log.info( "erreur initiale (t = %f): ", estim[0])
log.info( "  sur x (en mm): %f", (estim[1].xRobot - xx[0]) * 1000.)
log.info( "  sur y (en mm): %f", (estim[1].yRobot - yy[0]) * 1000.)
log.info( "  en cap (deg) : %f", np.degrees(betweenMinusPiAndPlusPi( estim[1].hRobot - hh[0] )))
log.info("covariance :\n%s", repr(estim[1].covariance))
log.info( "**********************************************")

estimates = []
estimates.append( kfloc.getBestEstimate() ) 

#===============================================================================
# Simulateur de LRF
#===============================================================================
lrfsim = LRFSimulator.LRFSimulator()
lrfsim.sigma = params.simu_cfg["sigmaLRF"]

# Positionnement des balises
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

kfloc.setBeacons( lrfsim.objects )

#===============================================================================
# Affichage initial
#===============================================================================
fig = plt.figure(figsize=(20,10))
plt.ion()
plt.hold(True)

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
    
if params.visu_cfg["zoom"]:
  ax.axis([np.mean(xx)-0.3, np.mean(xx)+0.3, np.mean(yy)-0.2, np.mean(yy)+0.2])


if params.visu_cfg["arrowTrue"]:
  xArrowBeg = xx[0]
  yArrowBeg = yy[0]
  xArrowEnd = 0.1 * np.cos(hh[0])
  yArrowEnd = 0.1 * np.sin(hh[0])
  arrow = plt.Arrow(xArrowBeg, yArrowBeg, xArrowEnd, yArrowEnd, alpha=0.8, width=0.01, color="grey")
  ax.add_patch(arrow)


if params.visu_cfg["arrowInit"]:
  xArrowBeg = kfloc.X[0,0]
  yArrowBeg = kfloc.X[1,0]
  xArrowEnd = 0.1 * np.cos(kfloc.X[2,0])
  yArrowEnd = 0.1 * np.sin(kfloc.X[2,0])
  arrow = plt.Arrow(xArrowBeg, yArrowBeg, xArrowEnd, yArrowEnd, alpha=0.8, width=0.01, color="red")
  ax.add_patch(arrow)
  
if params.visu_cfg["ellipseInit"]:
  estim = Estimate()
  estim.xRobot = kfloc.X[0,0]
  estim.yRobot = kfloc.X[1,0]
  estim.hRobot = kfloc.X[2,0]
  estim.covariance = kfloc.P
  xy, width, height, angle = getEllipseParametersFromEstimate(estim)
  ellipse = mpatches.Ellipse(xy, width, height, angle, alpha=0.3, ec="red", fc = "none")
  ax.add_patch(ellipse)
xOldEstim = kfloc.X[0,0]
yOldEstim = kfloc.X[1,0]

if params.visu_cfg["arrowInit"] and params.visu_cfg["arrowTrue"]:
  ax.plot( [kfloc.X[0,0], xx[0]], [kfloc.X[1,0], yy[0]], ':k' )
  
if params.visu_cfg["zoom"]:
  ax.axis([np.mean(xx)-0.3, np.mean(xx)+0.3, np.mean(yy)-0.2, np.mean(yy)+0.2])
else:
  ax.axis([-1.6, 1.6, -1.1, 1.1])
plt.title("time: 0.0")
plt.draw()
  
if params.visu_cfg["save"]:
  import os
  dirName = os.path.join(os.path.expanduser("~") , repr(graine) + "_" + repr(graine_))
  if not os.path.exists(dirName):
    os.mkdir(dirName)
  else:
    for f in os.listdir(dirName):
      os.remove(os.path.join(dirName, f))
  
  plt.savefig(os.path.join(dirName, "0_0.000" + ".png"))

#===============================================================================
# En avant !
#===============================================================================
needRedraw = False
for i, time in enumerate(tt):
  
  # on regarde s'il faut faire quelque chose
  if i == 0:
    doOdo = False
    doLrf = False
  else:
    doOdo = (math.floor(tt[i] / params.simu_cfg["odoTimeStep"]) > math.floor(tt[i-1] / params.simu_cfg["odoTimeStep"]) )
    doLrf = (math.floor(tt[i] /0.1) > math.floor(tt[i-1] / 0.1) )
    
  doOdo = doOdo and params.simu_cfg["odoSimu"]
  doLrf = doLrf and params.simu_cfg["lrfSimu"]
  
  # si on "reçoit" des données odo
  if doOdo:
    log.info("==============================================")
    log.info("time: %f => ODO", time)
    
    # simu des odos
    vxOdo = np.sum(vx[i-102:i+1]) / 103.
    vyOdo = np.sum(vy[i-102:i+1]) / 103.
    vhOdo = betweenMinusPiAndPlusPi(np.sum(vh[i-102:i+1]) / 103.)
    log.debug("-----------------------")
    log.debug("vxOdo: %f m/s", vxOdo)
    log.debug("vyOdo: %f m/s", vyOdo)
    log.debug("vhOdo: %f deg/s", np.degrees(vhOdo))
    ov = OdoVelocity()
    ov.vx = random.normalvariate(vxOdo, params.simu_cfg["sigmaTransOdoVelocity"])
    ov.vy = random.normalvariate(vyOdo, params.simu_cfg["sigmaTransOdoVelocity"])
    ov.vh = random.normalvariate(vhOdo, params.simu_cfg["sigmaRotOdoVelocity"])
    
    # Estimation de postion via les odos
    kfloc.newOdoVelocity(time, ov)
  
    estim = kfloc.getBestEstimate()
    log.debug("-----------------------")
    log.debug( "erreur après odos (t = %f):", estim[0])
    log.debug( "  sur x (en mm): %f", (estim[1].xRobot - xx[i]) * 1000.)
    log.debug( "  sur y (en mm): %f", (estim[1].yRobot - yy[i]) * 1000.)
    log.debug( "  en cap (deg) : %f", np.degrees(betweenMinusPiAndPlusPi( estim[1].hRobot - hh[i] )))
    log.debug("covariance :\n%s", repr(estim[1].covariance))  
    
    # On stocke l'estimée
    estimates.append( kfloc.getBestEstimate() ) 
    
    # Affichage de l'estimée post odo
    
    if params.visu_cfg["arrowOdo"] or (doLrf and params.visu_cfg["arrowLrf"]):
      xArrowBeg = estim[1].xRobot
      yArrowBeg = estim[1].yRobot
      xArrowEnd = 0.1 * np.cos(estim[1].hRobot)
      yArrowEnd = 0.1 * np.sin(estim[1].hRobot)
      arrow = plt.Arrow(xArrowBeg, yArrowBeg, xArrowEnd, yArrowEnd, width=0.01, alpha=0.1, color="red")
      ax.add_patch(arrow)
      
      ax.plot( [xOldEstim, xArrowBeg], [yOldEstim, yArrowBeg], '-b' )
      xOldEstim = xArrowBeg
      yOldEstim = yArrowBeg
      needRedraw = True
      
    if params.visu_cfg["ellipseOdo"] or (doLrf and params.visu_cfg["ellipseLrf"]):
      xy, width, height, angle = getEllipseParametersFromEstimate(estim[1])
      ellipse = mpatches.Ellipse(xy, width, height, angle, alpha=0.1, color="red")
      ax.add_patch(ellipse)
      needRedraw = True
      
  
  # si on "reçoit" un scan
  if doLrf:
    log.info("==============================================")
    log.info("time: %f => LRF", time)
    
    estim = kfloc.getBestEstimate()
    log.debug("-----------------------")
    log.debug( "erreur avant scan (t = %f):", estim[0])
    log.debug( "  sur x (en mm): %f", (estim[1].xRobot - xx[i]) * 1000.)
    log.debug( "  sur y (en mm): %f", (estim[1].yRobot - yy[i]) * 1000.)
    log.debug( "  en cap (deg) : %f", np.degrees(betweenMinusPiAndPlusPi( estim[1].hRobot - hh[i] )))
    log.debug("covariance :\n%s", repr(estim[1].covariance))   
    
    # Sélection de la fenêtre de position
    tt_ = tt[i-680:i+1]
    xx_ = xx[i-680:i+1]
    yy_ = yy[i-680:i+1]
    hh_ = hh[i-680:i+1]
  
    # On calcule le scan
    scan = lrfsim.computeScan(tt_, xx_, yy_, hh_)
    kfloc.scanproc.setTrueStaticPositionForDebugOnly(xx[i], yy[i], hh[i])
  
    # Estimation de postion via le scan
    kfloc.newScan(time, scan)
  
  # Estimée apres le scan
    estims = kfloc.getLastEstimates()
    for estim in estims[:-1]:
      log.debug( "-----------------------")
      log.debug( "erreur en cours d'update (t = %f):", estim[0])
      log.debug( "  sur x (en mm): %f", (estim[1].xRobot - xx[i]) * 1000.)
      log.debug( "  sur y (en mm): %f", (estim[1].yRobot - yy[i]) * 1000.)
      log.debug( "en cap (deg) : %f", np.degrees(betweenMinusPiAndPlusPi( estim[1].hRobot - hh[i] )))
      log.debug("covariance :\n%s", repr(estim[1].covariance))
      
      if params.visu_cfg["arrowUpdateLrf"]:    
        xArrowBeg = estim[1].xRobot
        yArrowBeg = estim[1].yRobot
        xArrowEnd = 0.1 * np.cos(estim[1].hRobot)
        yArrowEnd = 0.1 * np.sin(estim[1].hRobot)
        arrow = plt.Arrow(xArrowBeg, yArrowBeg, xArrowEnd, yArrowEnd, width=0.005, alpha=0.3, color="green")
        ax.add_patch(arrow)
        ax.plot( [xOldEstim, xArrowBeg], [yOldEstim, yArrowBeg], '-b' )
        xOldEstim = xArrowBeg
        yOldEstim = yArrowBeg
        needRedraw = True
      
      if params.visu_cfg["ellipseUpdateLrf"]:
        xy, width, height, angle = getEllipseParametersFromEstimate(estim[1])
        ellipse = mpatches.Ellipse(xy, width, height, angle, alpha=0.3, ec="green", fc="none")
        ax.add_patch(ellipse)
        needRedraw = True
      
    estim = kfloc.getBestEstimate()
    log.info( "-----------------------")
    log.info( "erreur après scan (t = %f): ", estim[0])
    log.info( "  sur x (en mm): %f", (estim[1].xRobot - xx[i]) * 1000.)
    log.info( "  sur y (en mm): %f", (estim[1].yRobot - yy[i]) * 1000.)
    log.info( "  en cap (deg) : %f", np.degrees(betweenMinusPiAndPlusPi( estim[1].hRobot - hh[i] )))
    log.info("covariance :\n%s", repr(estim[1].covariance))
    
    # On stocke l'estimée
    estimates.append( kfloc.getBestEstimate() ) 
    
    if params.visu_cfg["arrowLrf"]:
      # Affichage de l'estimée post scan
      xArrowBeg = estim[1].xRobot
      yArrowBeg = estim[1].yRobot
      xArrowEnd = 0.1 * np.cos(estim[1].hRobot)
      yArrowEnd = 0.1 * np.sin(estim[1].hRobot)
      arrow = plt.Arrow(xArrowBeg, yArrowBeg, xArrowEnd, yArrowEnd, width=0.01, alpha=0.3, color="blue")
      ax.add_patch(arrow)
      ax.plot( [xOldEstim, xArrowBeg], [yOldEstim, yArrowBeg], '-b' )
      xOldEstim = xArrowBeg
      yOldEstim = yArrowBeg
      needRedraw = True
      
    if params.visu_cfg["ellipseLrf"]:
      xy, width, height, angle = getEllipseParametersFromEstimate(estim[1])
      ellipse = mpatches.Ellipse(xy, width, height, angle, alpha=0.1, color="blue")
      ax.add_patch(ellipse)
      needRedraw = True
      
    
    # scan
    if params.visu_cfg["scan"]:
      L = len(scan.range)
      for k in range(L):
        if scan.range[-1-k] > 0.:
          xImpact = xx[i-k] + np.cos(hh[i-k] + scan.theta[-1-k]) * scan.range[-1-k]
          yImpact = yy[i-k] + np.sin(hh[i-k] + scan.theta[-1-k]) * scan.range[-1-k]
          ax.plot( [xImpact] , [yImpact], 'xb' )
          ax.plot( [xx[i-k], xImpact] , [yy[i-k], yImpact], '--b' )
      
      # field of view
      ax.plot( [xx[i-L], xx[i-L] + np.cos(hh[i-L] + min(scan.theta))], [yy[i-L], yy[i-L] + np.sin(hh[i-L] + min(scan.theta))], '-m')
      ax.plot( [xx[i], xx[i] + np.cos(hh[i] + max(scan.theta))], [yy[i], yy[i] + np.sin(hh[i] + max(scan.theta))], '-m')
      needRedraw = True
      
         
    # Affichage de la véritée terrain
  if (doOdo and params.visu_cfg["arrowTrue"] and params.visu_cfg["arrowOdo"]) or (doLrf and params.visu_cfg["arrowTrue"] and params.visu_cfg["arrowLrf"]) :
    xArrowBeg = xx[i]
    yArrowBeg = yy[i]
    xArrowEnd = 0.1 * np.cos(hh[i])
    yArrowEnd = 0.1 * np.sin(hh[i])
    arrow = plt.Arrow(xArrowBeg, yArrowBeg, xArrowEnd, yArrowEnd, width=0.01, alpha=0.3, color="grey")
    ax.add_patch(arrow)
    
    ax.plot( [estim[1].xRobot, xx[i]], [estim[1].yRobot, yy[i]], ':k' )
    needRedraw = True
    
  if needRedraw:
    if params.visu_cfg["zoom"]:
      ax.axis([np.mean(xx)-0.3, np.mean(xx)+0.3, np.mean(yy)-0.2, np.mean(yy)+0.2])
    else:
      ax.axis([-1.6, 1.6, -1.1, 1.1])
    plt.title("time: " + str(round(time,4)))
    plt.draw()
    needRedraw = False
    if params.visu_cfg["save"]:
      plt.savefig(os.path.join(dirName, str(i) + "_" + str(round(time,4)) + ".png"))
    
    log.info("==============================================")
    
      

log.info("graine pour la position réelle :%d", graine)
log.info("graine pour la simulation :%d", graine_)


plt.show()
