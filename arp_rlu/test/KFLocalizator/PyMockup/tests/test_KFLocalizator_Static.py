# coding=utf-8
import sys, os
sys.path.append( "../../../../src/KFLocalizator/PyMockup" )

import json

import roslib; roslib.load_manifest('arp_rlu')
import rospy

import numpy as np
import random
import matplotlib.pyplot as plt
from matplotlib.collections import PatchCollection
import matplotlib.patches as mpatches

from KFLocalizator import *
from LRFSimulator import *

import params_KFLocalizator_Static as params

np.set_printoptions(precision=4)

xpIndex = 1

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

#graine = 153
#graine_ = 771


random.seed(graine)
rospy.loginfo("graine pour la position réelle :%d", graine)
rospy.loginfo("graine pour la simulation :%d", graine_)


#===============================================================================
# Initialization du KF
#===============================================================================
kfloc = KFLocalizator()
trueX = random.uniform( -1.3, 1.3)
trueY = random.uniform( -0.8, 0.8)
trueH = random.uniform( -np.pi, np.pi)

random.seed(graine_)

initialXPosition = trueX + random.normalvariate(0., params.simu_cfg["sigmaInitialPosition"])
initialYPosition = trueY + random.normalvariate(0., params.simu_cfg["sigmaInitialPosition"])
initialHeading = betweenMinusPiAndPlusPi(trueH + random.normalvariate(0., params.simu_cfg["sigmaInitialHeading"]))
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
radius = 0.04
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
# Export de la position initiale
#===============================================================================
dictInitPos = {}
dictInitPos["trueX"] = trueX
dictInitPos["trueY"] = trueY
dictInitPos["trueH"] = trueH
dictInitPos["initialXPosition"] = initialXPosition
dictInitPos["initialYPosition"] = initialYPosition
dictInitPos["initialHPosition"] = initialHeading
if not os.path.exists("static_"+ str(xpIndex)):
  os.mkdir("static_"+ str(xpIndex))
output = open("./static_"+ str(xpIndex) + "/initial_position.json", mode='w')
output.write(json.dumps(dictInitPos,indent=2,sort_keys=True))
output.close()

#===============================================================================
# Export des paramètres du KFLocalizator
#===============================================================================
dictKFParams = {}
dictKFParams["sigmaInitialPosition"] = params.simu_cfg["sigmaInitialPosition"]
dictKFParams["sigmaInitialHeading"] = params.simu_cfg["sigmaInitialHeading"]
dictKFParams["sigmaTransOdoVelocity"] = params.kf_cfg["sigmaTransOdoVelocity"]
dictKFParams["sigmaRotOdoVelocity"] = params.kf_cfg["sigmaRotOdoVelocity"]
dictKFParams["sigmaLaserRange"] = params.kf_cfg["sigmaLaserRange"]
dictKFParams["sigmaLaserAngle"] = params.kf_cfg["sigmaLaserAngle"]
dictKFParams["iekf_Nit"] = params.kf_cfg["iekf_cfg"]["Nit"]
dictKFParams["iekf_xThreshold"] = params.kf_cfg["iekf_cfg"]["threshold"][0,0]
dictKFParams["iekf_yThreshold"] = params.kf_cfg["iekf_cfg"]["threshold"][1,0]
dictKFParams["iekf_hThreshold"] = params.kf_cfg["iekf_cfg"]["threshold"][2,0]
dictKFParams["beacondetector_maxDistance"] = params.kf_cfg["scanproc_cfg"]["maxDistance"]
dictKFParams["beacondetector_rangeThreshold"] = params.kf_cfg["scanproc_cfg"]["thresholdRange"]
if not os.path.exists("static_"+ str(xpIndex)):
  os.mkdir("static_"+ str(xpIndex))
output = open("./static_"+ str(xpIndex) + "/kfl_params.json", mode='w')
output.write(json.dumps(dictKFParams,indent=2,sort_keys=True))
output.close()


#===============================================================================
# En avant !
#===============================================================================

time = 0.

rospy.loginfo("=======================")
rospy.loginfo("Position réelle :")
rospy.loginfo("  xPosition: %f", trueX)
rospy.loginfo("  yPosition: %f", trueY)
rospy.loginfo("  hPosition: %f", trueH)

rospy.loginfo("=======================")
rospy.loginfo("Etat initial :")
rospy.loginfo("  xPosition: %f", initialXPosition)
rospy.loginfo("  yPosition: %f", initialYPosition)
rospy.loginfo("  hPosition: %f", initialHeading)
rospy.loginfo("  vx: %f", 0.)
rospy.loginfo("  vy: %f", 0.)
rospy.loginfo("  vz: %f", 0.)

    
rospy.loginfo("erreur statique sur l'état initial (t = %f) :", time)
rospy.loginfo("  sur x (en mm): %f", (initialXPosition - trueX) * 1000.)
rospy.loginfo("  sur y (en mm): %f", (initialYPosition - trueY) * 1000.)
rospy.loginfo("  en cap (deg) : %f", betweenMinusPiAndPlusPi( initialHeading - trueH ) *180./np.pi)
rospy.loginfo("covariance : \n%s", repr(kfloc.P))

xOld = initialXPosition
yOld = initialYPosition

time = 0.01

for k in range(params.simu_cfg["Nscans"]):
    rospy.loginfo("==============================================")
    rospy.loginfo("==============================================")
    rospy.loginfo(" TOUR %d", k)
    #===============================================================================
    # On reste sur place quelques sec
    #===============================================================================
    odoDurationInSec = 0.1
    ov = OdoVelocity()
    for t in np.arange(time, time + odoDurationInSec, 0.01):
      ov.vx = random.normalvariate(0., params.simu_cfg["sigmaTransOdoVelocity"])
      ov.vy = random.normalvariate(0., params.simu_cfg["sigmaTransOdoVelocity"])
      ov.vh = random.normalvariate(0., params.simu_cfg["sigmaRotOdoVelocity"])
      
      dictOdo = {}
      dictOdo["t"] = t
      dictOdo["vx"] = ov.vx
      dictOdo["vy"] = ov.vy
      dictOdo["vh"] = ov.vh
      if not os.path.exists("static_"+ str(xpIndex)):
        os.mkdir("static_"+ str(xpIndex))
      output = open("./static_"+ str(xpIndex) + "/t_" + str(t) + "_odo.json", mode='w')
      output.write(json.dumps(dictOdo,indent=2,sort_keys=True))
      output.close()
      
      kfloc.newOdoVelocity(t, ov, params.kf_cfg["sigmaTransOdoVelocity"], params.kf_cfg["sigmaTransOdoVelocity"], params.kf_cfg["sigmaRotOdoVelocity"])
      
      estimOdo = kfloc.getBestEstimate()
      dictEstim = {}
      dictEstim["t"] = estimOdo[0]
      dictEstim["x"] = estimOdo[1].xRobot
      dictEstim["y"] = estimOdo[1].yRobot
      dictEstim["h"] = estimOdo[1].hRobot
      dictEstim["covariance"] = {}
      dictEstim["covariance"]["raw_0"] = [estimOdo[1].covariance[0,0], estimOdo[1].covariance[0,1], estimOdo[1].covariance[0,2]]
      dictEstim["covariance"]["raw_1"] = [estimOdo[1].covariance[1,0], estimOdo[1].covariance[1,1], estimOdo[1].covariance[1,2]]
      dictEstim["covariance"]["raw_2"] = [estimOdo[1].covariance[2,0], estimOdo[1].covariance[2,1], estimOdo[1].covariance[2,2]]
      if not os.path.exists("static_"+ str(xpIndex)):
        os.mkdir("static_"+ str(xpIndex))
      output = open("./static_"+ str(xpIndex) + "/t_" + str(t) + "_odo_estimate.json", mode='w')
      output.write(json.dumps(dictEstim,indent=2,sort_keys=True))
      output.close()
      
    time = time + odoDurationInSec
    
    #===============================================================================
    # Estimee odo avant le scan
    #===============================================================================
    estim1 = kfloc.getBestEstimate()
    rospy.loginfo("=======================")
    #print "Estimee via odo (t =",estim1[0],"): "
    #print "  xPosition:", estim1[1].xRobot
    #print "  yPosition:", estim1[1].yRobot
    #print "  hPosition:", estim1[1].hRobot
    #print "  vx:", estim1[1].velXRobot
    #print "  vy:", estim1[1].velYRobot
    #print "  vz:", estim1[1].velHRobot
    # print "Covariance avant le scan :"
    # print estim1[1].covariance
    
    rospy.loginfo( "erreur statique apres les odos :")
    rospy.loginfo( "  sur x (en mm): %f", (estim1[1].xRobot - trueX) * 1000.)
    rospy.loginfo( "  sur y (en mm): %f", (estim1[1].yRobot - trueY) * 1000.)
    rospy.loginfo( "  en cap (deg) : %f", betweenMinusPiAndPlusPi( estim1[1].hRobot - trueH ) *180./np.pi)
    rospy.loginfo("covariance :\n%s", repr(estim1[1].covariance))    
    
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
    
    if not os.path.exists("static_"+ str(xpIndex)):
        os.mkdir("static_"+ str(xpIndex))
    scan.export("./static_"+ str(xpIndex) + "/t_" + str(time) + "_scan.json")
    
    #===============================================================================
    # Estimee apres le scan
    #===============================================================================
    estims = kfloc.getLastEstimates()
    for estim2 in estims[:-1]:
      rospy.loginfo( "-----------------------")
      rospy.loginfo( "  Erreur statique post update (t = %f): ", estim2[0])
      rospy.loginfo( "    sur x (en mm): %f", (estim2[1].xRobot - trueX) * 1000.)
      rospy.loginfo( "    sur y (en mm): %f", (estim2[1].yRobot - trueY) * 1000.)
      rospy.loginfo( "    en cap (deg) : %f", betweenMinusPiAndPlusPi( estim2[1].hRobot - trueH ) *180./np.pi)
      rospy.loginfo("covariance :\n%s", repr(estim2[1].covariance))

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
    rospy.loginfo( "=======================")
    #print "Estimée via scan (t =",estim2[0],"): "
    #print "  xPosition:", estim2[1].xRobot
    #print "  yPosition:", estim2[1].yRobot
    #print "  hPosition:", estim2[1].hRobot
    #print "  vx:", estim2[1].velXRobot
    #print "  vy:", estim2[1].velYRobot
    #print "  vz:", estim2[1].velHRobot
    #print "Covariance apres le scan :"
    #print estim2[1].covariance )
    rospy.loginfo( "Erreur statique après scan :")
    rospy.loginfo( "  sur x (en mm): %f", (estim2[1].xRobot - trueX) * 1000.)
    rospy.loginfo( "  sur y (en mm): %f", (estim2[1].yRobot - trueY) * 1000.)
    rospy.loginfo( "  en cap (deg) : %f", betweenMinusPiAndPlusPi( estim2[1].hRobot - trueH ) *180./np.pi)
    rospy.loginfo("covariance :\n%s", repr(estim2[1].covariance))
    
    dictEstim = {}
    dictEstim["t"] = estim2[0]
    dictEstim["x"] = estim2[1].xRobot
    dictEstim["y"] = estim2[1].yRobot
    dictEstim["h"] = estim2[1].hRobot
    dictEstim["covariance"] = {}
    dictEstim["covariance"]["raw_0"] = [estim2[1].covariance[0,0], estim2[1].covariance[0,1], estim2[1].covariance[0,2]]
    dictEstim["covariance"]["raw_1"] = [estim2[1].covariance[1,0], estim2[1].covariance[1,1], estim2[1].covariance[1,2]]
    dictEstim["covariance"]["raw_2"] = [estim2[1].covariance[2,0], estim2[1].covariance[2,1], estim2[1].covariance[2,2]]
    if not os.path.exists("static_"+ str(xpIndex)):
      os.mkdir("static_"+ str(xpIndex))
    output = open("./static_"+ str(xpIndex) + "/t_" + str(time) + "_scan_estimate.json", mode='w')
    output.write(json.dumps(dictEstim,indent=2,sort_keys=True))
    output.close()

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
    

rospy.loginfo("graine pour la position réelle :%d", graine)
rospy.loginfo("graine pour la simulation :%d", graine_)

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
