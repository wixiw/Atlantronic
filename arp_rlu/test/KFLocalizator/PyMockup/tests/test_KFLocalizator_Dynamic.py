# coding=utf-8
import sys
sys.path.append( "../../../../src/KFLocalizator/PyMockup" )

import roslib; roslib.load_manifest('arp_rlu')
import rospy

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


rospy.loginfo("graine pour la position réelle :%d", graine)
rospy.loginfo("graine pour la simulation :%d", graine_)
rospy.loginfo("")


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
rospy.loginfo("")
rospy.loginfo( "==============================================")
rospy.loginfo( "position réelle initiale (t = %f):", tt[0])
rospy.loginfo( "  x (en m): %f", xx[0])
rospy.loginfo( "  y (en m): %f", yy[0])
rospy.loginfo( "  h (en deg): %f", np.degrees(betweenMinusPiAndPlusPi( hh[0] )))
rospy.loginfo( "erreur initiale (t = %f): ", estim[0])
rospy.loginfo( "  sur x (en mm): %f", (estim[1].xRobot - xx[0]) * 1000.)
rospy.loginfo( "  sur y (en mm): %f", (estim[1].yRobot - yy[0]) * 1000.)
rospy.loginfo( "  en cap (deg) : %f", np.degrees(betweenMinusPiAndPlusPi( estim[1].hRobot - hh[0] )))
rospy.loginfo("covariance :\n%s", repr(estim[1].covariance))
rospy.loginfo( "==============================================")
rospy.loginfo("")

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
obj2.yCenter = 1.
obj2.radius = radius
lrfsim.objects.append(obj2)
obj3 = Circle()
obj3.xCenter = -1.5
obj3.yCenter = -1.
obj3.radius = radius
lrfsim.objects.append(obj3)


#===============================================================================
# Affichage initial
#===============================================================================
fig = plt.figure(figsize=(20,10))
plt.ion()

ax = fig.add_subplot(111, aspect='equal')

if params.visu_cfg["zoom"]:
  ax.axis([np.mean(xx)-0.3, np.mean(xx)+0.3, np.mean(yy)-0.2, np.mean(yy)+0.2])
else:
  ax.axis([-1.6, 1.6, -1.1, 1.1])
  
plt.hold(True)

# table
ax.plot( [-1.5, -1.5, 1.5, 1.5, -1.5], [-1., 1., 1., -1., -1.], '-k')


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
  
  if doOdo or doLrf:
    rospy.loginfo("==============================================")
    rospy.loginfo("==============================================")
    rospy.loginfo( "position réelle (t = %f):", tt[i])
    rospy.loginfo( "  x (en m): %f", xx[i])
    rospy.loginfo( "  y (en m): %f", yy[i])
    rospy.loginfo( "  h (en deg): %f", np.degrees(betweenMinusPiAndPlusPi( hh[i] )))
  
  # si on "reçoit" des données odo
  if doOdo:
    rospy.loginfo("==============================================")
    rospy.loginfo("time: %f => ODO", time)
    
    if params.simu_cfg["virtualOdo"]:
      # simule des odos virtuels
      ov = OdoVelocity()
      sigmaXOdo = math.fabs(vx[i]) #params.simu_cfg["virtualSigmaTransOdoVelocity"]
      sigmaYOdo = math.fabs(vy[i]) #params.simu_cfg["virtualSigmaTransOdoVelocity"]
      sigmaHOdo = math.fabs(vh[i]) #params.simu_cfg["virtualSigmaRotOdoVelocity"]
      
    else:
      # simule des vrais odos
      vxOdo = np.sum(vx[i-102:i+1]) / 103.
      vyOdo = np.sum(vy[i-102:i+1]) / 103.
      vhOdo = np.sum(vh[i-102:i+1]) / 103.
      rospy.loginfo("-----------------------")
      rospy.loginfo("vxOdo: %f m/s", vxOdo)
      rospy.loginfo("vyOdo: %f m/s", vyOdo)
      rospy.loginfo("vhOdo: %f deg/s", np.degrees(vhOdo))
      sigmaXOdo = np.max(  [params.simu_cfg["minSigmaTransOdoVelocity"], params.simu_cfg["percentSigmaTransOdoVelocity"] * np.fabs(vxOdo)])
      sigmaYOdo = np.max( [params.simu_cfg["minSigmaTransOdoVelocity"], params.simu_cfg["percentSigmaTransOdoVelocity"] * np.fabs(vyOdo)])
      sigmaHOdo = np.max( [params.simu_cfg["minSigmaRotOdoVelocity"],   params.simu_cfg["percentSigmaRotOdoVelocity"]   * np.fabs(vhOdo)])
      rospy.loginfo("sigmaXOdo: %f m/s", sigmaXOdo)
      rospy.loginfo("sigmaYOdo: %f m/s", sigmaYOdo)
      rospy.loginfo("sigmaHOdo: %f deg/s", np.degrees(sigmaHOdo))
      ov = OdoVelocity()
      ov.vx = random.normalvariate(vxOdo, sigmaXOdo)
      ov.vy = random.normalvariate(vyOdo, sigmaYOdo)
      ov.vh = random.normalvariate(vhOdo, sigmaHOdo)
    
    # Estimation de postion via les odos
    kfloc.newOdoVelocity(time, ov, sigmaXOdo, sigmaYOdo, sigmaHOdo)
  
    estim = kfloc.getBestEstimate()
    rospy.loginfo("-----------------------")
    rospy.loginfo( "erreur après odos (t = %f):", estim[0])
    rospy.loginfo( "  sur x (en mm): %f", (estim[1].xRobot - xx[i]) * 1000.)
    rospy.loginfo( "  sur y (en mm): %f", (estim[1].yRobot - yy[i]) * 1000.)
    rospy.loginfo( "  en cap (deg) : %f", np.degrees(betweenMinusPiAndPlusPi( estim[1].hRobot - hh[i] )))
    rospy.loginfo("covariance :\n%s", repr(estim[1].covariance))  
    
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
      ellipse = mpatches.Ellipse(xy, width, height, angle, alpha=0.1, ec="red", fc="red")
      ax.add_patch(ellipse)
      needRedraw = True
      
  
  # si on "reçoit" un scan
  if doLrf:
    rospy.loginfo("==============================================")
    rospy.loginfo("time: %f => LRF", time)
    
    estim = kfloc.getBestEstimate()
    rospy.loginfo("-----------------------")
    rospy.loginfo( "erreur avant scan (t = %f):", estim[0])
    rospy.loginfo( "  sur x (en mm): %f", (estim[1].xRobot - xx[i]) * 1000.)
    rospy.loginfo( "  sur y (en mm): %f", (estim[1].yRobot - yy[i]) * 1000.)
    rospy.loginfo( "  en cap (deg) : %f", np.degrees(betweenMinusPiAndPlusPi( estim[1].hRobot - hh[i] )))
    rospy.loginfo("covariance :\n%s", repr(estim[1].covariance))   
    
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
      rospy.loginfo( "-----------------------")
      rospy.loginfo( "erreur en cours d'update (t = %f):", estim[0])
      rospy.loginfo( "  sur x (en mm): %f", (estim[1].xRobot - xx[i]) * 1000.)
      rospy.loginfo( "  sur y (en mm): %f", (estim[1].yRobot - yy[i]) * 1000.)
      rospy.loginfo( "en cap (deg) : %f", np.degrees(betweenMinusPiAndPlusPi( estim[1].hRobot - hh[i] )))
      rospy.loginfo("covariance :\n%s", repr(estim[1].covariance))
      
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
    rospy.loginfo( "-----------------------")
    rospy.loginfo( "erreur après scan (t = %f): ", estim[0])
    rospy.loginfo( "  sur x (en mm): %f", (estim[1].xRobot - xx[i]) * 1000.)
    rospy.loginfo( "  sur y (en mm): %f", (estim[1].yRobot - yy[i]) * 1000.)
    rospy.loginfo( "  en cap (deg) : %f", np.degrees(betweenMinusPiAndPlusPi( estim[1].hRobot - hh[i] )))
    rospy.loginfo("covariance :\n%s", repr(estim[1].covariance))
    
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
    
    rospy.loginfo("==============================================")
    
      

rospy.loginfo("graine pour la position réelle :%d", graine)
rospy.loginfo("graine pour la simulation :%d", graine_)


plt.show()