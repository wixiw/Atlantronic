# coding=utf-8
import sys
sys.path.append( "../../../../src/KFLocalizator/PyMockup" )
import unittest
import numpy as np
import random
import math

from BaseClasses import Scan
from BaseClasses import Circle
from BaseClasses import Segment

from LRFSimulator import *


class TestLRFSimulator(unittest.TestCase):
  def setUp(self):
    self.lrfsim = LRFSimulator()
    
  def testInit(self):
    self.assertEqual(self.lrfsim.sigma, 0.)
    
  def testRayTracerEmpty(self):
    x = random.uniform( -1.5, 1.5)
    y = random.uniform( -1.0, 1.0)
    a = random.uniform( -2. * np.pi, 2. * np.pi)
    self.lrfsim.objects = [ ]
    range, intersection = self.lrfsim.rayTracer(x, y, a)
    self.assertEqual( range, float('inf') )
    self.assertEqual( intersection, None )
    
  def testRayTracerOneCircle(self):
    x = random.uniform( -1.5, 1.5)
    y = random.uniform( -1.0, 1.0)
    obj = Circle()
    obj.xCenter = random.uniform( -1.5, 1.5)
    obj.yCenter = random.uniform( -1.0, 1.0)
    obj.radius  = 0.05
    while np.linalg.norm( np.array( [ [obj.xCenter - x], [obj.yCenter - y] ] )) < obj.radius:
      obj.xCenter = random.uniform( -1.5, 1.5)
      obj.yCenter = random.uniform( -1.0, 1.0)
    self.lrfsim.objects = [ obj ]
    a = math.atan2( obj.yCenter - y, obj.xCenter - x )
    range, intersection = self.lrfsim.rayTracer(x, y, a)
    self.assertAlmostEqual( range, np.linalg.norm( np.array( [ [obj.xCenter - x], [obj.yCenter - y] ] )) - obj.radius )
    self.assertTrue( np.array_equiv(intersection, np.array( [[x + math.cos(a) * range], [y + math.sin(a) * range]] )) )
    
  def testRayTracerTwoCircles1(self):
    x = 0.
    y = 0.
    a = 0.
    obj1 = Circle()
    obj1.xCenter = 1.
    obj1.yCenter = 0.
    obj1.radius  = 0.1
    obj2 = Circle()
    obj2.xCenter = 2.
    obj2.yCenter = 0.
    obj2.radius  = 0.2
    self.lrfsim.objects = [ obj1, obj2 ]
    range, intersection = self.lrfsim.rayTracer(x, y, a)
    self.assertEqual( range, 0.9 )
    self.assertTrue( np.array_equiv(intersection, np.array( [[0.9], [0.0]] )) )
    
  def testRayTracerTwoCircles2(self):
    x = 0.
    y = 0.
    a = 0.
    obj1 = Circle()
    obj1.xCenter = 1.
    obj1.yCenter = 0.
    obj1.radius  = 0.1
    obj2 = Circle()
    obj2.xCenter = 2.
    obj2.yCenter = 0.
    obj2.radius  = 0.2
    self.lrfsim.objects = [ obj1, obj2 ]
    range, intersection = self.lrfsim.rayTracer(x, y, a)
    self.assertEqual( range, 0.9 )
    self.assertTrue( np.array_equiv(intersection, np.array( [[0.9], [0.0]] )) )
    
  def testComputeScanStatic(self):
    x  = 0.1
    y  = -0.4
    a  = np.pi / 2.
    xx = np.ones( (1000) ) * x
    yy = np.ones( (1000) ) * y
    aa = np.ones( (1000) ) * a
    tt = np.arange( 0.0, 0.1, 0.0001)
    scan = self.lrfsim.computeScan(tt, xx, yy, aa)
    
  def testRayTracerOneSegment1(self):
    x = 0.
    y = 0.
    a = 0.
    obj = Segment()
    obj.A = np.array( [[1.],[-0.5]])
    obj.B = np.array( [[1.],[ 0.5]])
    self.lrfsim.objects = [ obj ]
    range, intersection = self.lrfsim.rayTracer(x, y, a)
    self.assertEqual( range, 1. )
    self.assertTrue( np.array_equiv(intersection, np.array( [[1.], [0.]] )) )
    
  def testRayTracerOneSegment2(self):
    x = 0.
    y = 0.
    a = np.pi / 2.
    obj = Segment()
    obj.A = np.array( [[-1.],[0.5]])
    obj.B = np.array( [[ 1.],[0.5]])
    self.lrfsim.objects = [ obj ]
    range, intersection = self.lrfsim.rayTracer(x, y, a)
    self.assertEqual( range, 0.5 )
    self.assertTrue( np.array_equiv(intersection, np.array( [[0.], [0.5]] )) )
    
  def testRayTracerOneSegment3(self):
    x = 0.
    y = 0.
    a = -np.pi / 2.
    obj = Segment()
    obj.A = np.array( [[-1.],[0.5]])
    obj.B = np.array( [[ 1.],[0.5]])
    self.lrfsim.objects = [ obj ]
    range, intersection = self.lrfsim.rayTracer(x, y, a)
    self.assertEqual( range, float('inf') )
    self.assertEqual( intersection, None )
    

if __name__ == '__main__':
    unittest.main()