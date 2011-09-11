# coding=utf-8
import sys
sys.path.append( "../../../../src/KFLocalizator/PyMockup" )
import unittest
from numpy import *
import random
import math
import logging

from kmeans import *

Ntests = 100

class TestKMeans(unittest.TestCase):
  def setUp(self):
    pass
    
  def testEmpty(self):
    pc = PointCloud()
    for i in range(Ntests):
      (pcFirst, pcSecond) = kmeans(pc, maxIt=10, dispTh=0.01)
      self.assertTrue(array_equiv( pcFirst.points, zeros((3,0)) ))
      self.assertTrue(array_equiv( pcSecond.points, zeros((3,0)) ))
      
  def testOnePoint(self):
    pc = PointCloud(1)
    for i in range(Ntests):
      (pcFirst, pcSecond) = kmeans(pc, maxIt=10, dispTh=0.01)
      self.assertTrue(array_equiv( pcFirst.points, pc.points ))
      self.assertTrue(array_equiv( pcSecond.points, zeros((3,0)) ))
    
  def testTwoPoints(self):
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
      
  def testFull(self):
    pc = PointCloud(5)
    pc.points[0,0:] = [ -1.4, -0.2, -1.2, -0.3, -0.31 ] 
    pc.points[1,0:] = [  1.2,  0.7,  1.1,  0.6,  0.6  ] 
    for i in range(Ntests):
      (pcFirst, pcSecond) = kmeans(pc, maxIt=10, dispTh=0.01)
      self.assertTrue( pcFirst.points.shape[1] == 2 or pcFirst.points.shape[1] == 3 )
      self.assertTrue( pcSecond.points.shape[1] == 2 or pcSecond.points.shape[1] == 3 )

if __name__ == '__main__':
    logging.basicConfig(level=logging.ERROR) #level=logging.DEBUG  
    unittest.main()