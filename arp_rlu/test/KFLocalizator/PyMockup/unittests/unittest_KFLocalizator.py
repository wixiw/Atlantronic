import sys
sys.path.append( "../../../../src/KFLocalizator/PyMockup" )
import unittest
from numpy import *
import random
import math

from KFLocalizator import *

class TestKFLocalizator(unittest.TestCase):
  def setUp(self):
    self.kfloc = KFLocalizator()
    
  def testInit(self):
    self.assertEqual(self.kfloc.odoVelBuf, [])
    self.assertEqual(self.kfloc.estimateBuf, [])
    self.assertEqual(self.kfloc.timeBuf, [])
    self.assertEqual(self.kfloc.X, [])
    self.assertEqual(self.kfloc.P, [])
    self.assertEqual(self.kfloc.A, [])
    self.assertEqual(self.kfloc.Q, [])
    self.assertEqual(self.kfloc.B, [])
    self.assertEqual(self.kfloc.H, [])
    self.assertEqual(self.kfloc.R, [])
    
  def testInitialize(self):
    self.kfloc.initialize(1000,
                          0.,
                          0.,
                          0.,
                          0.,
                          0.,
                          0.,
                          0.,
                          0.)
    self.assertTrue( array_equiv(self.kfloc.odoVelBuf.data,   [None] * 1000 ))
    self.assertTrue( array_equiv(self.kfloc.estimateBuf.data, [None] * 1000 ))
    self.assertTrue( array_equiv(self.kfloc.timeBuf.data,     [None] * 1000 ))
    
    
    
if __name__ == '__main__':
  unittest.main()