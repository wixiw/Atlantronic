from numpy import *
import math
import random

from KalmanFilter import *
from BaseClasses import *
from ScanProcessor import *

class KFLocalizator:
  def __init__(self):
    self.timeBuf = []
    self.odoVelBuf = []
    self.estimateBuf = []
    
    self.scanproc = ScanProcessor()
    
    # mean state estimate
    # X[0,0] is xRobot
    # X[1,0] is yRobot
    # X[2,0] is thetaRobot
    # X[3,0] is temporal derivative of xRobot
    # X[4,0] is temporal derivative of yRobot
    # X[5,0] is temporal derivative of thetaRobot
    self.X = []
    # and its covariance (6x6 matrix)
    self.P = [] 
    
    # prediction model (6x6 matrix)
    self.A = []
    # and its process noise covariance matrix.  (6x6 matrix)
    self.Q = []
    
    # input effect matrix (6x6 matrix)
    self.B = []

    # measurement noise covariance matrix (1x1 matrix)
    self.R = []
    
  def initialize(self, currentTime, N, 
                       initialXPosition, initialYPosition, initialHeading, 
                       sigmaInitialPosition, sigmaInitialHeading,
                       sigmaTransOdoVelocity, sigmaRotOdoVelocity, sigmaLaserRange, sigmaLaserAngle):
    self.timeBuf     = RingBuffer(N)
    self.timeBuf.append(currentTime)
    self.odoVelBuf   = RingBuffer(N)
    self.estimateBuf = RingBuffer(N)
    self.X = array([[initialXPosition], 
                    [initialYPosition], 
                    [initialHeading], 
                    [0.], 
                    [0.],
                    [0.]])
    self.P = diag((sigmaInitialPosition,
                   sigmaInitialPosition,
                   sigmaInitialHeading,
                   0.,
                   0.,
                   0.))
    dt = 0.0001
    self.A = array([[1., 0., 0., dt, 0., 0.],
                    [0., 1., 0., 0., dt, 0.],
                    [0., 0., 1., 0., 0., dt],
                    [0., 0., 0., 0., 0., 0.],
                    [0., 0., 0., 0., 0., 0.],
                    [0., 0., 0., 0., 0., 0.]])
    self.Q = diag((0.,
                   0.,
                   0.,
                   sigmaTransOdoVelocity,
                   sigmaTransOdoVelocity,
                   sigmaRotOdoVelocity))
    self.B = zeros( (6,6) )
    
    self.R = array( [[sigmaLaserRange], [sigmaLaserAngle]] )
    
  def predict(self, currentT, ov, dt):
    U = array([[0.], 
               [0.], 
               [0.], 
               [ov.vx], 
               [ov.vy],
               [ov.va]])
    self.A = array([[1., 0., 0., dt, 0., 0.],
                    [0., 1., 0., 0., dt, 0.],
                    [0., 0., 1., 0., 0., dt],
                    [0., 0., 0., 0., 0., 0.],
                    [0., 0., 0., 0., 0., 0.],
                    [0., 0., 0., 0., 0., 0.]])
    (self.X, self.P) = kf_predict(self.X, self.P, self.A, self.Q, self.B, U)
  
  def newOdoVelocity(self, currentT, ov):
      
    dt = currentT - self.timeBuf.getNewest()
    predict(currentT, ov, dt)
    
    estim = Estimate()
    estim.xRobot = self.X[0,0]
    estim.yRobot = self.X[1,0]
    estim.hRobot = self.X[2,0]
    estim.velXRobot = self.X[3,0]
    estim.velYRobot = self.X[4,0]
    estim.velHRobot = self.X[5,0]
    estim.covariance = self.P
    
    self.timeBuf.append(currentT)
    self.odoVelBuf.append(ov)
    self.estimateBuf.append(estim)
    
  # for internal use only
  def backInThePast(self, tCurrent, duration = 0.1, deltaT = 0.0001):
    tt = arange(tCurrent-duration, tCurrent, deltaT)
    
    tt_ = []
    xx_ = []
    yy_ = []
    hh_ = []
    vvx_ = []
    vvy_ = []
    vvh_ = []
    covars_ = []
    for i in range(len(self.timeBuf.getAll())):  
      estim = self.estimateBuf.getAll()[i]
      if estim != None:
        tt_.append(self.timeBuf.getAll()[i])
        xx_.append(estim.xRobot)
        yy_.append(estim.yRobot)
        hh_.append(estim.hRobot)
        vvx_.append(estim.velXRobot)
        vvy_.append(estim.velYRobot)
        vvh_.append(estim.velHRobot)
        covars_.append(estim.covariance)

    xx = interp(tt, tt_, xx_)
    yy = interp(tt, tt_, yy_)
    hh = interp(tt, tt_, hh_)
    vvx = interp(tt, tt_, vvx_)
    vvy = interp(tt, tt_, vvy_)
    vvh = interp(tt, tt_, vvh_)
    covars = interpMatrix(tt, tt_, covars_)
    return (tt, xx, yy, hh, vvx, vvy, vvh, covars)
  
  
  def newScan(self, currentTime, scan):
    self.scanproc.setScan(scan)
    
    # back in the past
    (tt, xx, yy, hh, vvx, vvy, vvh, covars) = backInThePast(currentTime)
    
    # reinit self.X and self.P
    self.X = array([[xx[0]], 
                    [yy[0]], 
                    [hh[0]], 
                    [vvx[0]], 
                    [vvy[0]],
                    [vvh[0]]])
    self.P = covars[0]
    
    
    self.scanproc.findCluster(tt, xx, yy, hh)
    
    ov = OdoVelocity()
    
    # loop on time 
    for i in range(len(tt)):
      t = tt[i]
      (xBeacon, yBeacon) = self.scanproc.getBeacons(t)
      if xBeacon != None and yBeacon != None:
          Y = zeros((2,1))
          Y[0,0] = sqrt( (self.X[0,0] - xBeacon)**2 + (self.X[1,0] - yBeacon)**2 )
          Y[1,0] = atan2(yBeacon - self.X[1,0], xBeacon - self.X[0,0]) - self.X[2,0]
          H = zeros( (2,6) )
          H[0,0] = (self.X[0,0] - xBeacon) / sqrt( (self.X[0,0] - xBeacon)**2 + (self.X[1,0] - yBeacon)**2 )
          H[0,1] = (self.X[1,0] - yBeacon) / sqrt( (self.X[0,0] - xBeacon)**2 + (self.X[1,0] - yBeacon)**2 )
          H[0,2] = 0.
          H[1,0] = (self.X[1,0] - yBeacon) / ( (self.X[0,0] - xBeacon)**2 + (self.X[1,0] - yBeacon)**2 )
          H[1,1] = (self.X[0,0] - xBeacon) / ( (self.X[0,0] - xBeacon)**2 + (self.X[1,0] - yBeacon)**2 )
          H[1,2] = -1.
          (self.X, self.P, K,IM,IS) = kf_update(self.X, self.P, Y, H, self.R)
      
      ov = OdoVelocity()
      ov.vx = vvx[i]
      ov.vy = vvy[i]
      ov.vh = vvh[i]
      # TODO init dt
      predict(currentT, ov, dt)
      
    estim = Estimate()
    estim.xRobot = self.X[0,0]
    estim.yRobot = self.X[1,0]
    estim.hRobot = self.X[2,0]
    estim.velXRobot = self.X[3,0]
    estim.velYRobot = self.X[4,0]
    estim.velHRobot = self.X[5,0]
    estim.covariance = self.P
  
    self.timeBuf.append(t)
    self.odoVelBuf.append(ov)
    self.estimateBuf.append(estim)
  
  def getLastEstimate(self):
    return self.estimateBuf.getNewest()
  
