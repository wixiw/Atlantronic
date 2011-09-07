# coding=utf-8
import sys
sys.path.append( "../../../../src/KFLocalizator/PyMockup" )
import unittest
from numpy import *
import random

from BaseClasses import *

class TestBaseClasse(unittest.TestCase):
  def setUp(self):
    pass

  def testBetweenZeroAndTwoPi(self):
    for k in range(100):
      angle1 = random.uniform( -100., 100)
      angle2 = betweenZeroAndTwoPi(angle1)
      self.assertTrue( angle2 >= 0. )
      self.assertTrue( angle2 < 2.*pi )
      self.assertAlmostEqual(sin(angle1), sin(angle2))
      self.assertAlmostEqual(cos(angle1), cos(angle2))
  
  def testBetweenMinusPiAndPlusPi(self):
    for k in range(100):
      angle1 = random.uniform( -100., 100)
      angle2 = betweenMinusPiAndPlusPi(angle1)
      self.assertTrue( angle2 <= pi )
      self.assertTrue( angle2 >= -pi )
      self.assertAlmostEqual(sin(angle1), sin(angle2))
      self.assertAlmostEqual(cos(angle1), cos(angle2))

  def testInterp1d(self):
    tp = [ 1.0, 2.0, 3.0 ]
    yp = [0.0, 1.0, 3.0]
    tt = [ 0.0, 1.5, 2.5, 3.0, 4.0]
    y = interp1d(tt, tp, yp)
    self.assertTrue( array_equiv( y, array([-1.0, 0.5, 2.0, 3.0, 5.0]) ))
    
  def testInterpMatrix1(self):
    tp = arange(0., 1.0, 0.2)
    yp = []
    for i in range(len(tp)):
      yp.append( zeros( (3,4) ) )
    tt = arange(0., 1.0, 0.1)
    
    y = interpMatrix(tt, tp, yp)
    
    for i in range(len(tp)):
      self.assertTrue(array_equiv( y[i], zeros((3,4)) ))
    
  def testInterpMatrix2(self):
    tp = arange(0., 1.0, 0.5)
    yp = []
    yp.append( zeros( (2,3) ) )
    yp.append( array([[1., 2., 3.],
                      [4., 5., 6.]]) )
    tt = arange(0., 0.75, 0.25, )
    tt = [ -0.5, 0., 0.25, 0.5, 1. ]
    
    y = interpMatrix(tt, tp, yp)
    
    self.assertTrue(array_equiv( y[0], array([[-1, -2., -3], [-4., -5., -6.]]) ))
    self.assertTrue(array_equiv( y[1], zeros((2,3)) ))
    self.assertTrue(array_equiv( y[2], array([[0.5, 1., 1.5], [2., 2.5, 3.]]) ))
    self.assertTrue(array_equiv( y[3], array([[1., 2., 3.], [4., 5., 6.]]) ))
    self.assertTrue(array_equiv( y[4], array([[2., 4., 6.], [8., 10., 12.]]) ))
    
  def testMedianFilter1(self):
    mf = MedianFilter(3)
    
    v = [5.]
    med = mf.getMedian(v)
    self.assertAlmostEqual( med, 5.)
    
    v = [5., -1.]
    med = mf.getMedian(v)
    self.assertAlmostEqual( med, -1.)
    
    v = [0., 1., 2.]
    med = mf.getMedian(v)
    self.assertAlmostEqual( med, 1.)
    
    v = [0., -1., 2.]
    med = mf.getMedian(v)
    self.assertAlmostEqual( med, 0.)
    
    v = [0., -1., 0.5]
    med = mf.getMedian(v)
    self.assertAlmostEqual( med, 0.)
    
    v = [2.2, -1., 0.5]
    med = mf.getMedian(v)
    self.assertAlmostEqual( med, 0.5)
    
    v = [0., 1., 2., 3.]
    med = mf.getMedian(v)
    self.assertAlmostEqual( med, 1.)
    
    v = [3., -1., 8., -3., 5.]
    med = mf.getMedian(v)
    self.assertAlmostEqual( med, 3.)
    
  def testMedianFilter2(self):
    raw = zeros( (2,3) )
    raw[1,0:] = [3., -1, 5]
    mf = MedianFilter(3)
    filtscan = mf.compute(raw)
    self.assertTrue(array_equiv( filtscan, array([[0., 0., 0.], [3., 3., 5.]]) ))
    
  def testMedianFilter3(self):
    raw = zeros( (2,8) )
    raw[1,0:] = [3., -1, 5., 8., 4., 7., 5., 6.]
    mf = MedianFilter(3)
    filtscan = mf.compute(raw)
    self.assertTrue(array_equiv( filtscan, array([[0.,0.,0.,0.,0.,0.,0.,0.], [3., 3., 5., 5., 7., 5., 6., 6.]]) ))
    
  def testMedianFilter4(self):
    raw = zeros( (2,8) )
    raw[1,0:] = [3., -1, 5, 8., 4., 7., 5., 6.]
    mf = MedianFilter(4)
    filtscan = mf.compute(raw)
    self.assertTrue(array_equiv( filtscan, array([[0.,0.,0.,0.,0.,0.,0.,0.], [3., 3., 4., 5., 5., 5., 5., 6.]]) ))
    
  def testMedianFilter5(self):
    raw = zeros( (2,8) )
    raw[1,0:] = [3., -1, 5, 8., 4., 7., 5., 6.]
    mf = MedianFilter(5)
    filtscan = mf.compute(raw)
    self.assertTrue(array_equiv( filtscan, array([[0.,0.,0.,0.,0.,0.,0.,0.], [3., -1., 4., 5., 5., 6., 5., 6.]]) ))

  def testComputePointCloud1(self):
    scan = Scan()
    scan.theta = zeros( (4) )
    scan.range = zeros( (4) )
    tt = zeros( (4) )
    xx = zeros( (4) )
    yy = zeros( (4) )
    hh = zeros( (4) )
    pc = PointCloud()
    pc.fromScan(scan, tt, xx, yy, hh)
    self.assertTrue(array_equiv( pc.points, zeros( (2,4)) ))
    self.assertTrue(array_equiv( pc.tt, zeros( (4)) ))
    
  def testComputePointCloud2(self):
    scan = Scan()
    scan.theta = zeros( (4) )
    scan.theta[0:] = [0., pi/2., pi, -pi/2.]
    scan.range = zeros( (4) )
    scan.range[0:] = [1.0, 0.5, 2.0, 3.0]
    tt = zeros( (4) )
    xx = ones( (4) ) * 0.3
    yy = ones( (4) ) * -0.1
    hh = ones( (4) ) * pi/2.
    pc = PointCloud()
    pc.fromScan(scan, tt, xx, yy, hh)
    self.assertTrue(allclose( pc.points, array( [[ 0.3, -0.2, 0.3, 3.3 ], [ 0.9, -0.1,-2.1,-0.1]]) ))
    self.assertTrue(allclose( pc.tt, zeros( (4)) ))

    
if __name__ == '__main__':
    unittest.main()