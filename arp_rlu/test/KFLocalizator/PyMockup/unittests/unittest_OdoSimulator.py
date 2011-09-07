# coding=utf-8
import sys
sys.path.append( "../../../../src/KFLocalizator/PyMockup" )
import unittest
from numpy import *
import random

from OdoSimulator import *

class TestOdoSimulator(unittest.TestCase):
  def setUp(self):
    self.odosim = OdoSimulator()
    
  def testInit(self):
    self.assertEqual(self.odosim.sigma, 0.)
    
  def testStatic(self):
    dx = zeros( (100) )
    dy = zeros( (100) )
    da = zeros( (100) )
    t  = arange( 0.0, 0.01, 0.0001)
    ov = self.odosim.computeOdoVelocity( dx, dy, da, t )
    self.assertEqual(ov.vx, 0.)
    self.assertEqual(ov.vy, 0.)
    self.assertEqual(ov.va, 0.)
    
  def testConstantVelocities(self):
    vx = random.uniform( -3., 3.)
    vy = random.uniform( -3., 3.)
    va = random.uniform( -2. * pi, 2. * pi)
    dx = ones( (100) ) * vx 
    dy = ones( (100) ) * vy
    da = ones( (100) ) * va
    t  = arange( 0.0, 0.01, 0.0001)
    ov = self.odosim.computeOdoVelocity( dx, dy, da, t )
    self.assertAlmostEqual(ov.vx, vx)
    self.assertAlmostEqual(ov.vy, vy)
    self.assertAlmostEqual(ov.va, va)
    
  def testConstantAccelerations(self):
    ax  = random.uniform( -3., 3.)
    ay  = random.uniform( -3., 3.)
    aa  = random.uniform( -2. * pi, 2. * pi)
    vx0 = random.uniform( -3., 3.)
    vy0 = random.uniform( -3., 3.)
    va0 = random.uniform( -2. * pi, 2. * pi)
    t  = arange( 0.0, 0.01, 0.0001)
    dx = vx0 + t * ax
    dy = vy0 + t * ay
    da = va0 + t * aa
    ov = self.odosim.computeOdoVelocity( dx, dy, da, t )
    self.assertAlmostEqual(ov.vx, (vx0 + vx0 + 0.0099 * ax)/2. )
    self.assertAlmostEqual(ov.vy, (vy0 + vy0 + 0.0099 * ay)/2. )
    self.assertAlmostEqual(ov.va, (va0 + va0 + 0.0099 * aa)/2. )


if __name__ == '__main__':
    unittest.main()

