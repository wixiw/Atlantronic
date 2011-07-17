import sys
sys.path.append( "../../../../src/KFLocalizator/PyMockup" )
import unittest
from numpy import *
import random
import math

from BaseClasses import Scan
from BaseClasses import Object

from LRFSimulator import *


class TestLRFSimulator(unittest.TestCase):
  def setUp(self):
    self.lrfsim = LRFSimulator()
    
  def testInit(self):
    self.assertEqual(self.lrfsim.sigma, 0.)
    
  def testRayTracerEmpty(self):
    x = random.uniform( -1.5, 1.5)
    y = random.uniform( -1.0, 1.0)
    a = random.uniform( -2. * pi, 2. * pi)
    self.lrfsim.objects = [ ]
    range, intersection = self.lrfsim.rayTracer(x, y, a)
    self.assertEqual( range, float('inf') )
    self.assertEqual( intersection, None )
    
  def testRayTracerOneCircle(self):
    x = random.uniform( -1.5, 1.5)
    y = random.uniform( -1.0, 1.0)
    obj = Object()
    obj.xCenter = random.uniform( -1.5, 1.5)
    obj.yCenter = random.uniform( -1.0, 1.0)
    obj.radius  = 0.05
    while linalg.norm( array( [ [obj.xCenter - x], [obj.yCenter - y] ] )) < obj.radius:
      obj.xCenter = random.uniform( -1.5, 1.5)
      obj.yCenter = random.uniform( -1.0, 1.0)
    self.lrfsim.objects = [ obj ]
    a = math.atan2( obj.yCenter - y, obj.xCenter - x )
    range, intersection = self.lrfsim.rayTracer(x, y, a)
    self.assertAlmostEqual( range, linalg.norm( array( [ [obj.xCenter - x], [obj.yCenter - y] ] )) - obj.radius )
    self.assertTrue( array_equiv(intersection, array( [[x + cos(a) * range], [y + sin(a) * range]] )) )
    
  def testRayTracerTwoCircles1(self):
    x = 0.
    y = 0.
    a = 0.
    obj1 = Object()
    obj1.xCenter = 1.
    obj1.yCenter = 0.
    obj1.radius  = 0.1
    obj2 = Object()
    obj2.xCenter = 2.
    obj2.yCenter = 0.
    obj2.radius  = 0.2
    self.lrfsim.objects = [ obj1, obj2 ]
    range, intersection = self.lrfsim.rayTracer(x, y, a)
    self.assertEqual( range, 0.9 )
    self.assertTrue( array_equiv(intersection, array( [[0.9], [0.0]] )) )
    
  def testRayTracerTwoCircles2(self):
    x = 0.
    y = 0.
    a = 0.
    obj1 = Object()
    obj1.xCenter = 1.
    obj1.yCenter = 0.
    obj1.radius  = 0.1
    obj2 = Object()
    obj2.xCenter = 2.
    obj2.yCenter = 0.
    obj2.radius  = 0.2
    self.lrfsim.objects = [ obj1, obj2 ]
    range, intersection = self.lrfsim.rayTracer(x, y, a)
    self.assertEqual( range, 0.9 )
    self.assertTrue( array_equiv(intersection, array( [[0.9], [0.0]] )) )
    
  def testComputeScanStatic(self):
    x  = 0.1
    y  = -0.4
    a  = pi / 2.
    xx = ones( (1000) ) * x
    yy = ones( (1000) ) * y
    aa = ones( (1000) ) * a
    tt = arange( 0.0, 0.1, 0.0001)
    scan = self.lrfsim.computeScan(xx, yy, aa, tt)
    
    

if __name__ == '__main__':
    unittest.main()