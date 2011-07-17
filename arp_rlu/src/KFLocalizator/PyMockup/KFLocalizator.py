from numpy import *
import math
import random

from KalmanFilter import *
from BaseClasses import *
from ScanProcessor import *

class KFLocalizator:
  def __init__(self):
    self.odoVelBuf = []
    self.estimateBuf = []
    self.timeBuf = []
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
    
    # control input. (6x1 matrix)
    U = []
    
    # measurement : range of beacon (1x1 matrix)
    Y = []
    
    # measurement model (matrix 1x6)
    self.H = [] 
    # and its  measurement noise covariance matrix (1x1 matrix)
    self.R = []
    
  def initialize(self, N, 
                       initialXPosition, initialYPosition, initialHeading, 
                       sigmaInitialPosition, sigmaInitialHeading,
                       sigmaTransOdoVelocity, sigmaRotOdoVelocity, sigmaLaserRange):
    self.odoVelBuf   = RingBuffer(N)
    self.estimateBuf = RingBuffer(N)
    self.timeBuf     = RingBuffer(N)
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
    
    self.R = array( [[sigmaLaserRange]] )
  
  def newOdoVelocity(self, t, ov):
    U = array([[0.], 
               [0.], 
               [0.], 
               [ov.vx], 
               [ov.vy],
               [ov.va]])
    (self.X, self.P) = kf_predict(self.X, self.P, self.A, self.Q, self.B, U)
    
    self.timeBuf.append(t)
    self.odoVelBuf.append(ov)
    estim = Estimate()
    estim.xRobot = self.X[0,0]
    estim.yRobot = self.X[1,0]
    estim.hRobot = self.X[2,0]
    estim.velXRobot = self.X[3,0]
    estim.velYRobot = self.X[4,0]
    estim.velHRobot = self.X[5,0]
    estim.covariance = self.P
    self.estimateBuf.append(estim)
  
  def newScan(self, t, scan):
    self.scanproc.setScan(scan)
    
    # back in the past
    # TODO init tt, 
    # TODO reinit self.X and self.P
    # TODO init xx, yy and hh
    # TODO init vvx, vvy, vvh
    
    self.scanproc.findCluster(xx, yy, hh)
    
    
    # loop on time 
    for i in range(len(tt)):
      t = tt[i]
      (xBeacon, yBeacon) = self.scanproc.getBeacons(t)
      if xBeacon != None:
          # TODO init Y
          (self.X, self.P, K,IM,IS) = kf_update(self.X, self.P, Y, self.H, self.R)
      
      vx = vvx[i]
      vy = vvy[i]
      vh = vvh[i]
      U = array([[0.], [0.], [0.],  [vx],  [vy],  [vh]])
      (self.X, self.P) = kf_predict(self.X, self.P, self.A, self.Q, self.B, U)
  
  def getLastEstimate(self):
    return self.estimateBuf.getNewest()
  
