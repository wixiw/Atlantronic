# coding=utf-8
import sys
sys.path.append( "../../../../src/KFLocalizator/PyMockup" )
import unittest
import numpy as np
import math
import random
import logging

from Scan import Scan, MedianFilter
import BaseMethods

class TestMedianFilter(unittest.TestCase):
  def setUp(self):
    pass
  

  def testGetMedian(self):
    mf = MedianFilter(3)
    
    v = [5.]
    med = BaseMethods.getMedian(v)
    self.assertAlmostEqual( med, 5.)
    
    v = [5., -1.]
    med = BaseMethods.getMedian(v)
    self.assertAlmostEqual( med, -1.)
    
    v = [0., 1., 2.]
    med = BaseMethods.getMedian(v)
    self.assertAlmostEqual( med, 1.)
    
    v = [0., -1., 2.]
    med = BaseMethods.getMedian(v)
    self.assertAlmostEqual( med, 0.)
    
    v = [0., -1., 0.5]
    med = BaseMethods.getMedian(v)
    self.assertAlmostEqual( med, 0.)
    
    v = [2.2, -1., 0.5]
    med = BaseMethods.getMedian(v)
    self.assertAlmostEqual( med, 0.5)
    
    v = [0., 1., 2., 3.]
    med = BaseMethods.getMedian(v)
    self.assertAlmostEqual( med, 1.)
    
    v = [3., -1., 8., -3., 5.]
    med = BaseMethods.getMedian(v)
    self.assertAlmostEqual( med, 3.)
    
  def testMedianFilter2(self):
    raw = np.zeros( (3) )
    raw[0:] = [3., -1, 5]
    mf = MedianFilter(3)
    filt = mf.compute(raw)
    self.assertTrue(np.array_equiv( filt, np.array([3., 3., 5.]) ))
    
  def testMedianFilter3(self):
    raw = np.zeros( (8) )
    raw[0:] = [3., -1, 5., 8., 4., 7., 5., 6.]
    mf = MedianFilter(3)
    filt = mf.compute(raw)
    self.assertTrue(np.array_equiv( filt, np.array([3., 3., 5., 5., 7., 5., 6., 6.]) ))
    
  def testMedianFilter4(self):
    raw = np.zeros( (8) )
    raw[0:] = [3., -1, 5, 8., 4., 7., 5., 6.]
    mf = MedianFilter(4)
    filt = mf.compute(raw)
    self.assertTrue(np.array_equiv( filt, np.array([3., 3., 4., 5., 5., 5., 5., 6.]) ))
    
  def testMedianFilter5(self):
    raw = np.zeros( (8) )
    raw[0:] = [3., -1, 5, 8., 4., 7., 5., 6.]
    mf = MedianFilter(5)
    filt = mf.compute(raw)
    self.assertTrue(np.array_equiv( filt, np.array([3., -1., 4., 5., 5., 6., 5., 6.]) ))
    
  def testIntegration1(self):
    raw = Scan(8)
    raw.range = np.array( [3., -1, 5., 8., 4., 7., 5., 6.] )
    raw.doMedianFiltering(3)
    self.assertTrue(np.array_equiv( raw.range, np.array([3., 3., 5., 5., 7., 5., 6., 6.]) ))
    
  def testIntegration2(self):
    raw = Scan(8)
    raw.range = np.array( [3., -1, 5., 8., 4., 7., 5., 6.] )
    raw.doMedianFiltering(4)
    self.assertTrue(np.array_equiv( raw.range, np.array([3., 3., 4., 5., 5., 5., 5., 6.]) ))
    
  def testIntegration3(self):
    raw = Scan(8)
    raw.range = np.array( [3., -1, 5., 8., 4., 7., 5., 6.] )
    raw.doMedianFiltering(5)
    self.assertTrue(np.array_equiv( raw.range, np.array([3., -1., 4., 5., 5., 6., 5., 6.]) ))

    
if __name__ == '__main__':
#    logging.basicConfig(level=logging.ERROR) 
    logging.basicConfig(level=logging.DEBUG)  
    unittest.main()