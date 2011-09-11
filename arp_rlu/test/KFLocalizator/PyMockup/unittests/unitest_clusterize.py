# coding=utf-8
import sys
sys.path.append( "../../../../src/KFLocalizator/PyMockup" )
import unittest
from numpy import *
import random
import math
import logging

from clusterize import *

Ntests = 100

class TestClusterize(unittest.TestCase):
  def setUp(self):
    self.params = ClusterizeParams()
    
  def testEmpty(self):
    pc = PointCloud()
    for i in range(Ntests):
      result = clusterize(pc, self.params)
      self.assertTrue(array_equiv( result, [] ))
    
  def testSmall1(self):
    pc = PointCloud(2)
    pc.points[0,0:] = [ -1.4, -0.2 ] 
    pc.points[1,0:] = [  1.2,  0.7 ]
    for i in range(Ntests):
      result = clusterize(pc, self.params)
      self.assertTrue(array_equiv( result, [] ))
      
  def testSmall2(self):
    pc = PointCloud(4)
    pc.points[0,0:] = [ -1.4, -0.2,  1.7, -1.2 ] 
    pc.points[1,0:] = [  1.2,  0.7,  0.2,  3.7 ]
    for i in range(Ntests):
      result = clusterize(pc, self.params)
      self.assertTrue(array_equiv( result, [] ))
      
  def testFull1(self):
    pc = PointCloud(9)
    pc.points[0,0:] = [ -1.4, -0.2, -0.2, -1.3, -0.3, -0.31, -1.2, -1.2,  -1.3 ] 
    pc.points[1,0:] = [  1.2,  0.7,  0.6,  1.1,  0.6,  0.6,   1.1,  1.2,   1.2 ]
    result = clusterize(pc, self.params)
    for i in range(len(result)):
      self.assertEqual( len(result), 1 )
      
  def testFull2(self):
    pc = PointCloud(18)
    pc.points[0,0:] = [ -1.4, -0.2, -0.2, -1.3, -0.3, -0.31, -1.2, -1.2,  -1.3, -2.4, -1.2, -1.2, -2.3, -1.3, -1.31, -2.2, -2.2,  -2.3 ] 
    pc.points[1,0:] = [  1.2,  0.7,  0.6,  1.1,  0.6,  0.6,   1.1,  1.2,   1.2,  2.2, -1.7, -1.6,  2.1, -1.6, -1.6,   2.1,  2.2,   2.2 ]
    pc.points[2,0:] = range(18)
    result = clusterize(pc, self.params)
    self.assertEqual( len(result), 2 )

      
  def testEmpty(self):
    pc = PointCloud(2)
    pc.points[0,0:] = [ -1.4, -0.2 ] 
    pc.points[1,0:] = [  1.2,  0.7 ] 
    for i in range(Ntests):
      (pcFirst, pcSecond) = kmeans(pc, maxIt=10, dispTh=0.01)
      if pcFirst.points[0,0] == pc.points[0,0]:
        self.assertTrue(array_equiv( pcFirst.points[0:,0], pc.points[0:,0] ))
        self.assertTrue(array_equiv( pcSecond.points[0:,0], pc.points[0:,1] ))
      else:
        self.assertTrue(array_equiv( pcFirst.points[0:,0], pc.points[0:,1] ))
        self.assertTrue(array_equiv( pcSecond.points[0:,0], pc.points[0:,0] ))
    
if __name__ == '__main__':
    logging.basicConfig(level=logging.ERROR)
#    logging.basicConfig(level=logging.DEBUG)
    unittest.main()