import sys
sys.path.append( "../../../../src/KFLocalizator/PyMockup" )
import unittest
from numpy import *
import random
import math
import logging

from ScanProcessor2 import *

class TestScanProcessor2(unittest.TestCase):
  def setUp(self):
    self.scanproc = ScanProcessor2()
    
    
  def testKMeans1(self):
    pc = PointCloud()
    (pcFirst, pcSecond) = self.scanproc._ScanProcessor2__kmeans(pc)
    self.assertTrue(array_equiv( pcFirst.points, zeros((2,0)) ))
    self.assertTrue(array_equiv( pcFirst.tt, zeros((0)) ))
    self.assertTrue(array_equiv( pcSecond.points, zeros((2,0)) ))
    self.assertTrue(array_equiv( pcSecond.tt, zeros((0)) ))
    
  def testKMeans2(self):
    pc = PointCloud(1)
    (pcFirst, pcSecond) = self.scanproc._ScanProcessor2__kmeans(pc)
    self.assertTrue(array_equiv( pcFirst.points, pc.points ))
    self.assertTrue(array_equiv( pcFirst.tt, pc.tt ))
    self.assertTrue(array_equiv( pcSecond.points, zeros((2,0)) ))
    self.assertTrue(array_equiv( pcSecond.tt, zeros((0)) ))
    
  def testKMeans3(self):
    pc = PointCloud(2)
    pc.points[0,0:] = [ -1.4, -0.2 ] 
    pc.points[1,0:] = [  1.2,  0.7 ] 
    (pcFirst, pcSecond) = self.scanproc._ScanProcessor2__kmeans(pc)
    if pcFirst.points[0,0] == pc.points[0,0]:
      self.assertTrue(array_equiv( pcFirst.points[0:,0], pc.points[0:,0] ))
      self.assertTrue(array_equiv( pcSecond.points[0:,0], pc.points[0:,1] ))
    else:
      self.assertTrue(array_equiv( pcFirst.points[0:,0], pc.points[0:,1] ))
      self.assertTrue(array_equiv( pcSecond.points[0:,0], pc.points[0:,0] ))
      
  def testKMeans4(self):
    pc = PointCloud(5)
    pc.points[0,0:] = [ -1.4, -0.2, -1.2, -0.3, -0.31 ] 
    pc.points[1,0:] = [  1.2,  0.7,  1.1,  0.6,  0.6  ] 
    (pcFirst, pcSecond) = self.scanproc._ScanProcessor2__kmeans(pc)
    self.assertTrue( pcFirst.points.shape[1] == 2 or pcFirst.points.shape[1] == 3 )
    self.assertTrue( pcSecond.points.shape[1] == 2 or pcSecond.points.shape[1] == 3 )

if __name__ == '__main__':
    logging.basicConfig(level=logging.DEBUG)
    unittest.main()