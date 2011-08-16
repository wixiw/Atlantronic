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
    
    
if __name__ == '__main__':
    unittest.main()