# coding=utf-8
import roslib; roslib.load_manifest('arp_rlu')
import rospy

import numpy as np
import math
import random

from KalmanFilter import *
from BaseClasses import *
#from EnhancedScanProcessor import *
from ScanProcessor import *

class KFLocalizator:
  def __init__(self):
    self.buffer = []
    
    self.scanproc = ScanProcessor()
#    self.scanproc = EnhancedScanProcessor()
    self.givePerfectLRFMeasures = False
    
    # mean state estimate
    # X[0,0] is xRobot
    # X[1,0] is yRobot
    # X[2,0] is thetaRobot
    self.X = []
    # and its covariance (3x3 matrix)
    self.P = [] 
    
    
    # command noise covariance matrix.  (3x3 matrix)
    self.Qv = [] 
    self.Q = []
    
    # Max number of iterations (in case of IEKF only)
    self.Nit = 10
    # threshold use to stop iteration (in case of IEKF only)
    self.threshold = np.array([ [0.01], [0.01], [0.01]])
    
  def initialize(self, currentTime, N, 
                       initialXPosition, initialYPosition, initialHeading, 
                       sigmaInitialPosition, sigmaInitialHeading,
                       sigmaTransOdoVelocity, sigmaRotOdoVelocity, sigmaLaserRange, sigmaLaserAngle, sigmaSegmentHeading):
    self.X = np.array([[initialXPosition], 
                    [initialYPosition], 
                    [initialHeading]])
    self.P = np.diag((sigmaInitialPosition**2,
                   sigmaInitialPosition**2,
                   sigmaInitialHeading**2))
    estim = Estimate()
    estim.xRobot = self.X[0,0]
    estim.yRobot = self.X[1,0]
    estim.hRobot = self.X[2,0]
    estim.velXRobot = 0.
    estim.velYRobot = 0.
    estim.velHRobot = 0.
    estim.covariance = self.P
    self.buffer = RingBuffer(N)
    self.buffer.append([currentTime, estim])
    
    self.sigmaXOdo = sigmaTransOdoVelocity
    self.sigmaYOdo = sigmaTransOdoVelocity
    self.sigmaHOdo = sigmaRotOdoVelocity
    self.sigmaLaserRange = sigmaLaserRange
    self.sigmaLaserAngle = sigmaLaserAngle
    self.sigmaSegmentHeading = sigmaSegmentHeading
    
    
  def predict(self, currentT, ov, dt):
    U = np.array([[ov.vx], 
               [ov.vy],
               [ov.vh]])
    A = np.array([[1., 0., 0.],
               [0., 1., 0.],
               [0., 0., 1.]])
    
    w,v = np.linalg.eig(self.P)
    Q_ = np.sqrt(w) + np.array( [self.sigmaXOdo * dt, self.sigmaYOdo * dt, self.sigmaHOdo * dt] ) 
    Q = np.dot( v , np.dot( np.diag( Q_**2 - w ), v.transpose()))
    
    (self.X, self.P) = kf_predict(self.X, self.P, A, Q, np.diag((dt,dt,dt)), U)
  
  def newOdoVelocity(self, currentT, ov, sigmaXOdo_, sigmaYOdo_, sigmaHOdo_ ):
    last = self.buffer.getNewest()
    if last == None:
      return False
    if last[0] == None:
      return False
    
    dt = currentT - last[0]
    
    self.sigmaXOdo = sigmaXOdo_
    self.sigmaYOdo = sigmaYOdo_
    self.sigmaHOdo = sigmaHOdo_

    self.predict(currentT, ov, dt)
    
    estim = Estimate()
    estim.xRobot = self.X[0,0]
    estim.yRobot = self.X[1,0]
    estim.hRobot = self.X[2,0]
    estim.velXRobot = ov.vx
    estim.velYRobot = ov.vy
    estim.velHRobot = ov.vh
    estim.covariance = self.P
    
    self.buffer.append( [ currentT, estim ] ) 
    return True
    
  # for internal use only
  def backInThePast(self, tCurrent, duration = 681. * 0.1 / 1024., deltaT = 0.1 / 1024.):
    tt_ = []
    xx_ = []
    yy_ = []
    hh_ = []
    vvx_ = []
    vvy_ = []
    vvh_ = []
    covars_ = []
    for x in self.buffer.getAll():
      if x != None:
        t = x[0]
        estim = x[1]
        if estim != None:
          tt_.append(t)
          xx_.append(estim.xRobot)
          yy_.append(estim.yRobot)
          hh_.append(estim.hRobot)
          vvx_.append(estim.velXRobot)
          vvy_.append(estim.velYRobot)
          vvh_.append(estim.velHRobot)
          covars_.append(estim.covariance)
          
    if len(tt_) == 0:
      return (tt_, xx_, yy_, hh_, vvx_, vvy_, vvh_, covars_)

    tt = np.arange(tCurrent-duration, tCurrent, deltaT)
    tt = tt[:681]
    xx = interp1d(tt, tt_, xx_)
    yy = interp1d(tt, tt_, yy_)
    hh = interp1d(tt, tt_, hh_)
    vvx = interp1d(tt, tt_, vvx_)
    vvy = interp1d(tt, tt_, vvy_)
    vvh = interp1d(tt, tt_, vvh_)
    covars = interpMatrix(tt, tt_, covars_)
    return (tt, xx, yy, hh, vvx, vvy, vvh, covars)
  
  
  def newScan(self, currentTime, scan):
    self.scanproc.setScan(scan)
    
    # back in the past
    duration = 681. * 0.1 / 1024.
    dt = 0.1 / 1024.
    (tt, xx, yy, hh, vvx, vvy, vvh, covars) = self.backInThePast(currentTime, duration, dt)
    if len(tt) == 0:
      return False
  
    self.buffer.clear()
    
    # reinit self.X and self.P
    self.X = np.array([[xx[0]], 
                    [yy[0]], 
                    [hh[0]]])
    self.P = covars[0]
    rospy.loginfo("back in the past : covars:\n%s",repr(covars[0]))
    
    self.scanproc.findCluster(tt, xx, yy, hh)
#    self.scanproc.do(scan, tt, xx, yy, hh)
    
    
#    print "========================================="
    nbVisibleBeacons = 0
    
    rospy.loginfo("=======================")
    rospy.loginfo("nb of detected clusters: %d", len(self.scanproc.objects))
    rospy.loginfo("objects :")
    for o in self.scanproc.objects:
      rospy.loginfo(" " + o.__str__())
      
    if len(self.scanproc.objects) < 2:
      rospy.loginfo("Only one beacon has been seen")
    # loop on time 
    for i in range(len(tt)):
      t = tt[i]
      hBeacon = None
      (xBeacon, yBeacon, r, theta) = self.scanproc.getBeacons(t)

      if self.givePerfectLRFMeasures:
        (xBeacon, yBeacon, r, theta) = self.scanproc.getTrueBeacons(i)
        
      if xBeacon != None and yBeacon != None:
        
          # print "========================================="
          # print "xBeacon =", xBeacon
          # print "yBeacon =", yBeacon
          Y = np.zeros((2,1))
          Y[0,0] = r
          Y[1,0] = theta
          # print "KFLocalizator - newScan() : Y="; print Y
          def J(X):
            H = np.zeros( (2,3) )
            H[0,0] = (X[0,0] - xBeacon) / np.sqrt( (X[0,0] - xBeacon)**2 + (X[1,0] - yBeacon)**2 )
            H[0,1] = (X[1,0] - yBeacon) / np.sqrt( (X[0,0] - xBeacon)**2 + (X[1,0] - yBeacon)**2 )
            H[0,2] = 0.
            H[1,0] = (X[1,0] - yBeacon) / ( (X[0,0] - xBeacon)**2 + (X[1,0] - yBeacon)**2 )
            H[1,1] = (X[0,0] - xBeacon) / ( (X[0,0] - xBeacon)**2 + (X[1,0] - yBeacon)**2 )
            H[1,2] = -1.
            return H
        
          IM = np.zeros((2,1))
          IM[0,0] = np.sqrt((self.X[0,0] - xBeacon)**2 + (self.X[1,0] - yBeacon)**2 )
          IM[1,0] = betweenMinusPiAndPlusPi(math.atan2(yBeacon - self.X[1,0], xBeacon - self.X[0,0]) -  self.X[2,0])
          
          R = np.diag((self.sigmaLaserRange**2, 
                       self.sigmaLaserAngle**2))
          
          
          rospy.loginfo("---")
          rospy.loginfo("mesure : Y.r= %f  Y.theta= %f", r, theta)
          rospy.loginfo("simulée pre update: IM.r= %f  IM.theta= %f", IM[0,0], IM[1,0])
          rospy.loginfo("Y[0] - IM[0]= %f", (Y[0,0] - IM[0,0])*1000.)
            
#          rospy.loginfo( "-----------------------")
#          rospy.loginfo( "Estimée pre update:")
#          rospy.loginfo( "  sur x (en m):%f", self.X[0,0] )
#          rospy.loginfo( "  sur y (en m):%f", self.X[1,0] )
#          rospy.loginfo( "  en cap (deg) :%f", betweenMinusPiAndPlusPi(self.X[2,0]) * 180./np.pi)


#          (self.X, self.P, K,IM,IS) = ekf_update(self.X, self.P, Y, J(self.X), R, IM)
          rospy.loginfo("covariance pre update: \n%s", repr(self.P))
          (self.X, self.P, K,IM,IS, k) = iekf_update(self.X, self.P, Y, J, R, IM, self.Nit, self.threshold)
          rospy.loginfo("covariance post update: \n%s", repr(self.P))
          
          
          IM = np.zeros((2,1))
          IM[0,0] = np.sqrt((self.X[0,0] - xBeacon)**2 + (self.X[1,0] - yBeacon)**2 )
          IM[1,0] = betweenMinusPiAndPlusPi(math.atan2(yBeacon - self.X[1,0], xBeacon - self.X[0,0]) -  self.X[2,0])
          rospy.loginfo("simulée post update: IM.r= %f  IM.theta= %f", IM[0,0], IM[1,0])
          rospy.loginfo("Y[0] - IM[0]= %f", (Y[0,0] - IM[0,0])*1000.)
          
#          rospy.loginfo( "-----------------------")
#          rospy.loginfo( "Estimée post update:")
#          rospy.loginfo( "  sur x (en m):%f", self.X[0,0] )
#          rospy.loginfo( "  sur y (en m):%f", self.X[1,0] )
#          rospy.loginfo( "  en cap (deg) :%f", betweenMinusPiAndPlusPi(self.X[2,0]) * 180./np.pi)
          
          nbVisibleBeacons = nbVisibleBeacons + 1
          rospy.loginfo("xBeacon: %f  yBeacon: %f  hBeacon: %s", xBeacon, yBeacon, str(hBeacon))
          
          estim = Estimate()
          estim.xRobot = self.X[0,0]
          estim.yRobot = self.X[1,0]
          estim.hRobot = self.X[2,0]
          estim.covariance = self.P
          self.buffer.append([t, estim])
      
      ov = OdoVelocity()
      ov.vx = vvx[i]
      ov.vy = vvy[i]
      ov.vh = vvh[i]
      self.predict(t, ov, dt)
    
    rospy.loginfo("  ==> %d beacons have been seen", nbVisibleBeacons)
      
    estim = Estimate()
    estim.xRobot = self.X[0,0]
    estim.yRobot = self.X[1,0]
    estim.hRobot = self.X[2,0]
    estim.velXRobot = ov.vx
    estim.velYRobot = ov.vy
    estim.velHRobot = ov.vh
    estim.covariance = self.P
  
    self.buffer.append([t, estim])
    return True
  
  def getBestEstimate(self):
    return self.buffer.getNewest()

  def getLastEstimates(self):
    return self.buffer.getAllNoNone()

  def setBeacons(self, beacons):
    self.scanproc.beacons = beacons
  
