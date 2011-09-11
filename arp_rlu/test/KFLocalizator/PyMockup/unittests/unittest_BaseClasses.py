# coding=utf-8
import sys
sys.path.append( "../../../../src/KFLocalizator/PyMockup" )
import unittest
import numpy as np
import math
import random
import logging

from BaseClasses import *

class TestBaseClasses(unittest.TestCase):
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

  def testComputePointCloud1(self):
    scan = Scan()
    scan.theta = np.zeros( (4) )
    scan.range = np.zeros( (4) )
    tt = np.zeros( (4) )
    xx = np.zeros( (4) )
    yy = np.zeros( (4) )
    hh = np.zeros( (4) )
    pc = PointCloud()
    pc.fromScan(scan, tt, xx, yy, hh)
    self.assertTrue(np.array_equiv( pc.points, np.zeros( (3,4)) ))
    
  def testComputePointCloud2(self):
    scan = Scan()
    scan.theta = np.zeros( (4) )
    scan.theta[0:] = [0., np.pi/2., np.pi, -np.pi/2.]
    scan.range = np.zeros( (4) )
    scan.range[0:] = [1.0, 0.5, 2.0, 3.0]
    tt = np.zeros( (4) )
    xx = np.ones( (4) ) * 0.3
    yy = np.ones( (4) ) * -0.1
    hh = np.ones( (4) ) * np.pi/2.
    pc = PointCloud()
    pc.fromScan(scan, tt, xx, yy, hh)
    self.assertTrue(np.allclose( pc.points, np.array( [[ 0.3, -0.2, 0.3, 3.3 ], 
                                                       [ 0.9, -0.1,-2.1,-0.1 ],
                                                       [ 0. ,  0. , 0., 0.  ]]) ))

    
if __name__ == '__main__':
#    logging.basicConfig(level=logging.ERROR) 
    logging.basicConfig(level=logging.DEBUG)  
    unittest.main()