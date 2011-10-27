# coding=utf-8
import numpy as np
import math
import random

from KalmanFilter import *
from BaseClasses import *
from ScanProcessor import *
from ScanProcessor2 import *

class KFLocalizator:
  def __init__(self):
    self.buffer = []
    
#    self.scanproc = ScanProcessor()
    self.scanproc = ScanProcessor()
    
    # mean state estimate
    # X[0,0] is xRobot
    # X[1,0] is yRobot
    # X[2,0] is thetaRobot
    self.X = []
    # and its covariance (3x3 matrix)
    self.P = [] 
    
    # command noise covariance matrix.  (3x3 matrix)
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
    
    self.Q = np.diag((sigmaTransOdoVelocity**2,
                   sigmaTransOdoVelocity**2,
                   sigmaRotOdoVelocity**2))
    
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
    (self.X, self.P) = kf_predict(self.X, self.P, A, dt * self.Q, np.diag((dt,dt,dt)), U)
  
  def newOdoVelocity(self, currentT, ov):
    last = self.buffer.getNewest()
    if last == None:
      return False
    if last[0] == None:
      return False
  
    dt = currentT - last[0]
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
    log = logging.getLogger('newScan')
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
    log.debug("back in the past : covars:\n%s",repr(covars[0]))
    
    
    self.scanproc.findCluster(tt, xx, yy, hh)
#    self.scanproc.process(scan, tt, xx, yy, hh)
    
    
#    print "========================================="
    nbVisibleBeacons = 0
    
    log.debug("=======================")
    log.debug("nb of detected clusters: %d", len(self.scanproc.objects))
    log.debug("objects :")
    for o in self.scanproc.objects:
      log.debug(" " + o.__str__())
      
    if len(self.scanproc.objects) < 2:
      log.info("Only one beacon has been seen")
    # loop on time 
    for i in range(len(tt)):
      t = tt[i]
      heading = None
      hBeacon = None
      (xBeacon, yBeacon, r, theta) = self.scanproc.getBeacons(t)
#      (xBeacon, yBeacon, hBeacon, r, theta, heading) = self.scanproc.getBeacons(t, epsilon_time=tt[1]-tt[0])
#      (xBeacon, yBeacon, r, theta) = self.scanproc.getTrueBeacons(i)
      if xBeacon != None and yBeacon != None:
        
          if heading is None:
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
            
            
            log.debug("---")
            log.debug("mesure : Y.r= %f  Y.theta= %f", r, theta)
            log.debug("simulée pre update: IM.r= %f  IM.theta= %f", IM[0,0], IM[1,0])
            log.debug("Y[0] - IM[0]= %f", (Y[0,0] - IM[0,0])*1000.)
            
          else:
            # on voit le cap du segment
            log.debug(" heading OK :-)")
            Y = np.zeros((3,1))
            Y[0,0] = r
            Y[1,0] = theta
            Y[2,0] = heading
  
            def J(X):
              H = np.zeros( (3,3) )
              H[0,0] = (X[0,0] - xBeacon) / np.sqrt( (X[0,0] - xBeacon)**2 + (X[1,0] - yBeacon)**2 )
              H[0,1] = (X[1,0] - yBeacon) / np.sqrt( (X[0,0] - xBeacon)**2 + (X[1,0] - yBeacon)**2 )
              H[0,2] = 0.
              H[1,0] = (X[1,0] - yBeacon) / ( (X[0,0] - xBeacon)**2 + (X[1,0] - yBeacon)**2 )
              H[1,1] = (X[0,0] - xBeacon) / ( (X[0,0] - xBeacon)**2 + (X[1,0] - yBeacon)**2 )
              H[1,2] = -1.
              H[2,0] = (X[1,0] - yBeacon) / ( (X[0,0] - xBeacon)**2 + (X[1,0] - yBeacon)**2 )
              H[2,1] = (X[0,0] - xBeacon) / ( (X[0,0] - xBeacon)**2 + (X[1,0] - yBeacon)**2 )
              H[2,2] = 0.
              return H
            
            IM = np.zeros((3,1))
            IM[0,0] = np.sqrt((self.X[0,0] - xBeacon)**2 + (self.X[1,0] - yBeacon)**2 )
            IM[1,0] = betweenMinusPiAndPlusPi(math.atan2(yBeacon - self.X[1,0], xBeacon - self.X[0,0]) -  self.X[2,0])
            IM[2,0] = betweenMinusPiAndPlusPi(math.atan2(yBeacon - self.X[1,0], xBeacon - self.X[0,0]) -  hBeacon)
            # print "KFLocalizator - newScan() : IM="; print IM
            
            
            R = np.diag((self.sigmaLaserRange**2, 
                         self.sigmaLaserAngle**2,
                         self.sigmaSegmentHeading**2))
            
            log.debug("---")
            log.debug("mesure : Y.r= %f  Y.theta= %f  Y.beta= %f", r, theta, heading)
            log.debug("simulée : IM.r= %f  IM.theta= %f  IM.beta= %f", IM[0,0], IM[1,0], IM[2,0])
            log.debug("Y[0] - IM[0]= %f", (Y[0,0] - IM[0,0])*1000.)
            
#          log.debug( "-----------------------")
#          log.debug( "Estimée pre update:")
#          log.debug( "  sur x (en m):%f", self.X[0,0] )
#          log.debug( "  sur y (en m):%f", self.X[1,0] )
#          log.debug( "  en cap (deg) :%f", betweenMinusPiAndPlusPi(self.X[2,0]) * 180./np.pi)


#          (self.X, self.P, K,IM,IS) = ekf_update(self.X, self.P, Y, J(self.X), R, IM)
          log.debug("covariance pre update: \n%s", repr(self.P))
          (self.X, self.P, K,IM,IS, k) = iekf_update(self.X, self.P, Y, J, R, IM, self.Nit, self.threshold)
          log.debug("covariance post update: \n%s", repr(self.P))
          
          
          IM = np.zeros((2,1))
          IM[0,0] = np.sqrt((self.X[0,0] - xBeacon)**2 + (self.X[1,0] - yBeacon)**2 )
          IM[1,0] = betweenMinusPiAndPlusPi(math.atan2(yBeacon - self.X[1,0], xBeacon - self.X[0,0]) -  self.X[2,0])
          log.debug("simulée post update: IM.r= %f  IM.theta= %f", IM[0,0], IM[1,0])
          log.debug("Y[0] - IM[0]= %f", (Y[0,0] - IM[0,0])*1000.)
          
#          log.debug( "-----------------------")
#          log.debug( "Estimée post update:")
#          log.debug( "  sur x (en m):%f", self.X[0,0] )
#          log.debug( "  sur y (en m):%f", self.X[1,0] )
#          log.debug( "  en cap (deg) :%f", betweenMinusPiAndPlusPi(self.X[2,0]) * 180./np.pi)
          
          nbVisibleBeacons = nbVisibleBeacons + 1
          log.debug("xBeacon: %f  yBeacon: %f  hBeacon: %s", xBeacon, yBeacon, str(hBeacon))
          
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
    
    log.debug("  ==> %d beacons have been seen", nbVisibleBeacons)
      
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
  
