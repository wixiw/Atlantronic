# coding=utf-8
import sys
sys.path.append( "../../../../src/KFLocalizator/PyMockup" )
import unittest
import numpy as np
import math
import random
import logging

from BaseClasses import *

class TestBaseMethods(unittest.TestCase):
  def setUp(self):
    pass

  def testBetweenZeroAndTwoPi(self):
    for k in range(100):
      angle1 = random.uniform( -100., 100)
      angle2 = betweenZeroAndTwoPi(angle1)
      self.assertTrue( angle2 >= 0. )
      self.assertTrue( angle2 < 2.*np.pi )
      self.assertAlmostEqual(math.sin(angle1), math.sin(angle2))
      self.assertAlmostEqual(math.cos(angle1), math.cos(angle2))
  
  def testBetweenMinusPiAndPlusPi(self):
    for k in range(100):
      angle1 = random.uniform( -100., 100)
      angle2 = betweenMinusPiAndPlusPi(angle1)
      self.assertTrue( angle2 <= np.pi )
      self.assertTrue( angle2 >= -np.pi )
      self.assertAlmostEqual(math.sin(angle1), math.sin(angle2))
      self.assertAlmostEqual(math.cos(angle1), math.cos(angle2))

  def testInterp1d(self):
    tp = [ 1.0, 2.0, 3.0 ]
    yp = [0.0, 1.0, 3.0]
    tt = [ 0.0, 1.5, 2.5, 3.0, 4.0]
    y = interp1d(tt, tp, yp)
    self.assertTrue( np.array_equiv( y, np.array([-1.0, 0.5, 2.0, 3.0, 5.0]) ))
    
  def testInterpMatrix1(self):
    tp = np.arange(0., 1.0, 0.2)
    yp = []
    for i in range(len(tp)):
      yp.append( np.zeros( (3,4) ) )
    tt = np.arange(0., 1.0, 0.1)

    y = interpMatrix(tt, tp, yp)
    
    for i in range(len(tp)):
      self.assertTrue(np.array_equiv( y[i], np.zeros((3,4)) ))
    
  def testInterpMatrix2(self):
    tp = np.arange(0., 1.0, 0.5)
    yp = []
    yp.append( np.zeros( (2,3) ) )
    yp.append( np.array([[1., 2., 3.],
                      [4., 5., 6.]]) )
    tt = np.arange(0., 0.75, 0.25, )
    tt = [ -0.5, 0., 0.25, 0.5, 1. ]
    
    y = interpMatrix(tt, tp, yp)
    
    self.assertTrue(np.array_equiv( y[0], np.array([[-1, -2., -3], [-4., -5., -6.]]) ))
    self.assertTrue(np.array_equiv( y[1], np.zeros((2,3)) ))
    self.assertTrue(np.array_equiv( y[2], np.array([[0.5, 1., 1.5], [2., 2.5, 3.]]) ))
    self.assertTrue(np.array_equiv( y[3], np.array([[1., 2., 3.], [4., 5., 6.]]) ))
    self.assertTrue(np.array_equiv( y[4], np.array([[2., 4., 6.], [8., 10., 12.]]) ))

  def testGetEllipseParametersFromEstimate1(self):
    estim = Estimate()
    estim.xRobot = 10.
    estim.yRobot = -3.
    estim.covariance = np.diag((3., 2., 0.))
    xy, width, height, angle = getEllipseParametersFromEstimate(estim)
    self.assertAlmostEqual( xy[0], 10 )
    self.assertAlmostEqual( xy[1], -3 )
    self.assertAlmostEqual( width, 3. * math.sqrt(3.) )
    self.assertAlmostEqual( height, 3. * math.sqrt(2.) )
    self.assertAlmostEqual( angle, 0. )
    
    
if __name__ == '__main__':
    logging.basicConfig(level=logging.ERROR) 
#    logging.basicConfig(level=logging.DEBUG)  
    unittest.main()