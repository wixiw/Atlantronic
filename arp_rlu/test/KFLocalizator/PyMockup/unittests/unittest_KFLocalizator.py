# coding=utf-8
import sys
sys.path.append( "../../../../src/KFLocalizator/PyMockup" )
import unittest
from numpy import *
import random
import math

from KFLocalizator import *
from LRFSimulator import *

class TestKFLocalizator(unittest.TestCase):
  def setUp(self):
    self.kfloc = KFLocalizator()
    
#  def init(self):
#    self.initialXPosition = random.uniform( -1.5, 1.5)
#    self.initialYPosition = random.uniform( -1.0, 1.0)
#    self.initialHeading = random.uniform( -2. * pi, 2. * pi)
#    self.sigmaInitialPosition = 0.002
#    self.sigmaInitialHeading = 0.01
#    self.sigmaTransOdoVelocity = 0.01
#    self.sigmaRotOdoVelocity = 0.01
#    self.sigmaLaserRange = 0.01
#    self.sigmaLaserAngle = 0.0
#    self.kfloc.initialize(0.0,
#                          1000,
#                          self.initialXPosition, self.initialYPosition, self.initialHeading, 
#                          self.sigmaInitialPosition, self.sigmaInitialHeading,
#                          self.sigmaTransOdoVelocity, self.sigmaRotOdoVelocity, 
#                          self.sigmaLaserRange, self.sigmaLaserAngle)
#    
#  def testInitialize(self):
#    self.init()
#    estim = Estimate()
#    estim.xRobot = self.initialXPosition
#    estim.yRobot = self.initialYPosition
#    estim.hRobot = self.initialHeading
#    estim.velXRobot = 0.
#    estim.velYRobot = 0.
#    estim.velHRobot = 0.
#    estim.covariance = diag((self.sigmaInitialPosition, self.sigmaInitialPosition, self.sigmaInitialHeading, 0., 0., 0.))
#    buf = [ None ] * 1000
#    buf[-1] = [0.0, estim]
#    self.assertTrue( array_equiv(self.kfloc.buffer.data[0:998], buf[0:998] ))
#    self.assertEqual( self.kfloc.buffer.data[999][0], buf[999][0] )
#    self.assertEqual( self.kfloc.buffer.data[999][1].xRobot, buf[999][1].xRobot )
#    self.assertEqual( self.kfloc.buffer.data[999][1].yRobot, buf[999][1].yRobot )
#    self.assertEqual( self.kfloc.buffer.data[999][1].hRobot, buf[999][1].hRobot )
#    self.assertEqual( self.kfloc.buffer.data[999][1].velXRobot, buf[999][1].velXRobot )
#    self.assertEqual( self.kfloc.buffer.data[999][1].velYRobot, buf[999][1].velYRobot )
#    self.assertEqual( self.kfloc.buffer.data[999][1].velHRobot, buf[999][1].velHRobot )
#    self.assertTrue( array_equiv( self.kfloc.buffer.data[999][1].covariance, buf[999][1].covariance ))
#    
#    self.assertTrue( array_equiv( self.kfloc.X, array([[self.initialXPosition], 
#                                                       [self.initialYPosition], 
#                                                       [self.initialHeading], 
#                                                       [0.], 
#                                                       [0.],
#                                                       [0.]]) ))    
#    
#    self.assertTrue( array_equiv( self.kfloc.P, diag((self.sigmaInitialPosition,
#                                                      self.sigmaInitialPosition,
#                                                      self.sigmaInitialHeading,
#                                                      0.,
#                                                      0.,
#                                                      0.)) ))
#    
#    self.assertTrue( array_equiv( self.kfloc.Q, diag((0.,
#                                                      0.,
#                                                      0.,
#                                                      self.sigmaTransOdoVelocity,
#                                                      self.sigmaTransOdoVelocity,
#                                                      self.sigmaRotOdoVelocity)) ))
#    
#    self.assertTrue( array_equiv( self.kfloc.R, diag((self.sigmaLaserRange, self.sigmaLaserAngle))  ))
#    
#  def testGetBestEstimate(self):
#    self.init()
#    best = self.kfloc.getBestEstimate()
#    self.assertEqual( best[0], 0.0 )
#    self.assertEqual( best[1].xRobot, self.initialXPosition )
#    self.assertEqual( best[1].yRobot, self.initialYPosition )
#    self.assertEqual( best[1].hRobot, self.initialHeading )
#    self.assertEqual( best[1].velXRobot, 0.0 )
#    self.assertEqual( best[1].velYRobot, 0.0 )
#    self.assertEqual( best[1].velHRobot, 0.0 )
#    self.assertTrue( array_equiv( best[1].covariance, diag((self.sigmaInitialPosition, 
#                                                            self.sigmaInitialPosition, 
#                                                            self.sigmaInitialHeading, 
#                                                            0., 
#                                                            0., 
#                                                            0.)) ))
#    
#  def testOneOdoStep(self):
#    self.init() 
#    currentT = 0.1
#    ov = OdoVelocity()
#    ov.vx = 2.0
#    ov.vy = -1.0
#    ov.vh = 0.5
#    self.kfloc.newOdoVelocity(currentT, ov)
#    estim = self.kfloc.getBestEstimate()
#    self.assertEqual( estim[0], 0.1 )
#    self.assertEqual( estim[1].xRobot, self.initialXPosition )
#    self.assertEqual( estim[1].yRobot, self.initialYPosition )
#    self.assertEqual( estim[1].hRobot, self.initialHeading )
#    self.assertEqual( estim[1].velXRobot, ov.vx )
#    self.assertEqual( estim[1].velYRobot, ov.vy )
#    self.assertEqual( estim[1].velHRobot, ov.vh )
#     
#  def testTrivialMultiOdoSteps(self):
#    self.init() 
#    ov = OdoVelocity()
#    ov.vx = 0.
#    ov.vy = 0.
#    ov.vh = 0.
#    for t in arange(0., 5., 0.01):
#      self.kfloc.newOdoVelocity(t, ov)
#      estim = self.kfloc.getBestEstimate()
#      self.assertEqual( estim[0], t )
#      self.assertEqual( estim[1].xRobot, self.initialXPosition )
#      self.assertEqual( estim[1].yRobot, self.initialYPosition )
#      self.assertEqual( estim[1].hRobot, self.initialHeading )
#      self.assertEqual( estim[1].velXRobot, ov.vx )
#      self.assertEqual( estim[1].velYRobot, ov.vy )
#      self.assertEqual( estim[1].velHRobot, ov.vh )
#    
#===============================================================================
#  def testOneScanStep(self):
#    self.init() 
#    ov = OdoVelocity()
#    ov.vx = 0.
#    ov.vy = 0.
#    ov.vh = 0.
#    for t in arange(0., 5., 0.01):
#      self.kfloc.newOdoVelocity(t, ov)
#    
#    lrfsim = LRFSimulator()
#    # lrfsim.sigma = 0.01
# 
#    x = self.initialXPosition
#    y = self.initialYPosition
#    h = self.initialHeading
#    vh = 0.
#    tt = arange( 0.0, 0.1, 0.1 / 1024.)
#    xx = ones( (len(tt)) ) * x
#    yy = ones( (len(tt)) ) * y
#    hh = tt * vh + h
# 
#    # Add objects
#    nbObjects = 20
#    lrfsim.objects = []
#    for i in range(nbObjects):
#      obj = Object()
#      obj.xCenter = random.uniform( -1.5, 1.5)
#      obj.yCenter = random.uniform( -1.0, 1.0)
#      obj.radius  = random.uniform( 0.03, 0.1 )
#      while linalg.norm( array( [ [obj.xCenter - x], [obj.yCenter - y] ] )) < obj.radius:
#        obj.xCenter = random.uniform( -1.5, 1.5)
#        obj.yCenter = random.uniform( -1.0, 1.0)
#      lrfsim.objects.append(obj)
#    
#    # Compute scan
#    scan = lrfsim.computeScan(xx, yy, hh, tt)
#    
#    self.kfloc.setBeacons( lrfsim.objects )
#    self.kfloc.newScan(5., scan)
#    
#    estim = self.kfloc.getBestEstimate()
#    self.assertAlmostEqual( estim[0], 5. , places=3)
#    self.assertAlmostEqual( estim[1].xRobot, self.initialXPosition , places=3)
#    self.assertAlmostEqual( estim[1].yRobot, self.initialYPosition , places=3)
#    self.assertAlmostEqual( estim[1].hRobot, self.initialHeading , places=3)
#    self.assertAlmostEqual( estim[1].velXRobot, 0. , places=3)
#    self.assertAlmostEqual( estim[1].velYRobot, 0. , places=3)
#    self.assertAlmostEqual( estim[1].velHRobot, 0. , places=3)
#===============================================================================
    

if __name__ == '__main__':
  unittest.main()