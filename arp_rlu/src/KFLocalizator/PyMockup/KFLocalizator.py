# coding=utf-8
from numpy import *
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
    self.scanproc = ScanProcessor2()
    
    # mean state estimate
    # X[0,0] is xRobot
    # X[1,0] is yRobot
    # X[2,0] is thetaRobot
    self.X = []
    # and its covariance (3x3 matrix)
    self.P = [] 
    
    # command noise covariance matrix.  (3x3 matrix)
    self.Q = []

    
  def initialize(self, currentTime, N, 
                       initialXPosition, initialYPosition, initialHeading, 
                       sigmaInitialPosition, sigmaInitialHeading,
                       sigmaTransOdoVelocity, sigmaRotOdoVelocity, sigmaLaserRange, sigmaLaserAngle, sigmaSegmentHeading):
    self.X = array([[initialXPosition], 
                    [initialYPosition], 
                    [initialHeading]])
    self.P = diag((sigmaInitialPosition,
                   sigmaInitialPosition,
                   sigmaInitialHeading))
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
    
    self.Q = diag((sigmaTransOdoVelocity,
                   sigmaTransOdoVelocity,
                   sigmaRotOdoVelocity))
    
    self.sigmaLaserRange = sigmaLaserRange
    self.sigmaLaserAngle = sigmaLaserAngle
    self.sigmaSegmentHeading = sigmaSegmentHeading
    
    
  def predict(self, currentT, ov, dt):
    U = array([[ov.vx], 
               [ov.vy],
               [ov.vh]])
    A = array([[1., 0., 0.],
               [0., 1., 0.],
               [0., 0., 1.]])
    (self.X, self.P) = kf_predict(self.X, self.P, A, self.Q, diag((dt,dt,dt)), U)
  
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

    tt = arange(tCurrent-duration, tCurrent, deltaT)
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
#    self.scanproc.setScan(scan)
    
    # back in the past
    duration = 681. * 0.1 / 1024.
    dt = 0.1 / 1024.
    (tt, xx, yy, hh, vvx, vvy, vvh, covars) = self.backInThePast(currentTime, duration, dt)
    if len(tt) == 0:
      return False
  
    self.buffer.clear()
    
    # reinit self.X and self.P
    self.X = array([[xx[0]], 
                    [yy[0]], 
                    [hh[0]]])
    self.P = covars[0]
    
    
#    self.scanproc.findCluster(tt, xx, yy, hh)
    self.scanproc.process(scan, tt, xx, yy, hh)
    
    
#    print "========================================="
    nbVisibleBeacons = 0
    
    print "======================="
    print "nb of detected clusters:", len(self.scanproc.objects)
    print "objects :"
    for o in self.scanproc.objects:
      print " ",o
      
    
    # loop on time 
    for i in range(len(tt)):
      t = tt[i]
      heading = None
      hBeacon = None
#      (xBeacon, yBeacon, r, theta) = self.scanproc.getBeacons(t)
      (xBeacon, yBeacon, hBeacon, r, theta, heading) = self.scanproc.getBeacons(t, epsilon_time=tt[1]-tt[0])
#      heading = None
      #(xBeacon, yBeacon, r, theta) = self.scanproc.getTrueBeacons(i)
      if xBeacon != None and yBeacon != None:
        
          if heading is None:
            # print "========================================="
            # print "xBeacon =", xBeacon
            # print "yBeacon =", yBeacon
            Y = zeros((2,1))
            Y[0,0] = r
            Y[1,0] = theta
            # print "KFLocalizator - newScan() : Y="; print Y
            def J(X):
              H = zeros( (2,3) )
              H[0,0] = (X[0,0] - xBeacon) / sqrt( (X[0,0] - xBeacon)**2 + (X[1,0] - yBeacon)**2 )
              H[0,1] = (X[1,0] - yBeacon) / sqrt( (X[0,0] - xBeacon)**2 + (X[1,0] - yBeacon)**2 )
              H[0,2] = 0.
              H[1,0] = (X[1,0] - yBeacon) / ( (X[0,0] - xBeacon)**2 + (X[1,0] - yBeacon)**2 )
              H[1,1] = (X[0,0] - xBeacon) / ( (X[0,0] - xBeacon)**2 + (X[1,0] - yBeacon)**2 )
              H[1,2] = -1.
              return H
          
            # print "KFLocalizator - newScan() : H="; print H
            IM = zeros((2,1))
            IM[0,0] = sqrt((self.X[0,0] - xBeacon)**2 + (self.X[1,0] - yBeacon)**2 )
            IM[1,0] = betweenMinusPiAndPlusPi(math.atan2(yBeacon - self.X[1,0], xBeacon - self.X[0,0]) -  self.X[2,0])
            # print "KFLocalizator - newScan() : IM="; print IM
            
            R = diag((self.sigmaLaserRange, 
                      self.sigmaLaserAngle))
            
          else:
            # on voit le cap du segment
            print " heading OK :-)"
            Y = zeros((3,1))
            Y[0,0] = r
            Y[1,0] = theta
            Y[2,0] = heading
  
            def J(X):
              H = zeros( (3,3) )
              H[0,0] = (X[0,0] - xBeacon) / sqrt( (X[0,0] - xBeacon)**2 + (X[1,0] - yBeacon)**2 )
              H[0,1] = (X[1,0] - yBeacon) / sqrt( (X[0,0] - xBeacon)**2 + (X[1,0] - yBeacon)**2 )
              H[0,2] = 0.
              H[1,0] = (X[1,0] - yBeacon) / ( (X[0,0] - xBeacon)**2 + (X[1,0] - yBeacon)**2 )
              H[1,1] = (X[0,0] - xBeacon) / ( (X[0,0] - xBeacon)**2 + (X[1,0] - yBeacon)**2 )
              H[1,2] = -1.
              H[2,0] = (X[1,0] - yBeacon) / ( (X[0,0] - xBeacon)**2 + (X[1,0] - yBeacon)**2 )
              H[2,1] = (X[0,0] - xBeacon) / ( (X[0,0] - xBeacon)**2 + (X[1,0] - yBeacon)**2 )
              H[2,2] = 0.
              return H
            
            IM = zeros((3,1))
            IM[0,0] = sqrt((self.X[0,0] - xBeacon)**2 + (self.X[1,0] - yBeacon)**2 )
            IM[1,0] = betweenMinusPiAndPlusPi(math.atan2(yBeacon - self.X[1,0], xBeacon - self.X[0,0]) -  self.X[2,0])
            IM[2,0] = betweenMinusPiAndPlusPi(math.atan2(yBeacon - self.X[1,0], xBeacon - self.X[0,0]) -  hBeacon)
            # print "KFLocalizator - newScan() : IM="; print IM
            
            
            R = diag((self.sigmaLaserRange, 
                      self.sigmaLaserAngle,
                      self.sigmaSegmentHeading))
            
            print "---"
            print "mesure : Y.r=", r, "  Y.theta=", theta, "  Y.beta=", heading
            print "simulée : IM.r=", IM[0,0], "  IM.theta=", IM[1,0], "  IM.beta=", IM[2,0]
            print "Y[0] - IM[0]=", (Y[0,0] - IM[0,0])*1000.
            
#          print "--------------------"
#          print "estimée pre update :"
#          print "  sur x (en mm):", self.X[0,0] * 1000.
#          print "  sur y (en mm):", self.X[1,0] * 1000.
#          print "  en cap (deg) :", betweenMinusPiAndPlusPi(self.X[2,0]) * 180.*pi
#          print "---"
#          print "mesure : Y.r=", r, "  Y.theta=", theta
#          print "simulée : IM.r=", IM[0,0], "  IM.theta=", IM[1,0]
#          print "Y[0] - IM[0]=", (Y[0,0] - IM[0,0])*1000.
#          (self.X, self.P, K,IM,IS) = ekf_update(self.X, self.P, Y, J(self.X), R, IM)
            
          Nit = 10
          threshold = array([ [0.01], [0.01], [0.01]])
          (self.X, self.P, K,IM,IS, k) = iekf_update(self.X, self.P, Y, J, R, IM, Nit, threshold)
          
          
#          print "estimée post update :"
#          print "  sur x (en mm):", self.X[0,0] * 1000.
#          print "  sur y (en mm):", self.X[1,0] * 1000.
#          print "  en cap (deg) :", betweenMinusPiAndPlusPi(self.X[2,0]) * 180.*pi
          nbVisibleBeacons = nbVisibleBeacons + 1
          print "xBeacon:", xBeacon, "  yBeacon:", yBeacon, "  hBeacon:", str(hBeacon)
          
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
    
    print "  ==>",nbVisibleBeacons, "beacons have been seen"
      
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
  
