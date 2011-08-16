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
    self.assertEqual(self.kfloc.buffer, [])
    self.assertEqual(self.kfloc.X, [])
    self.assertEqual(self.kfloc.P, [])
    self.assertEqual(self.kfloc.Q, [])
    self.assertEqual(self.kfloc.B, [])
    self.assertEqual(self.kfloc.R, [])
    
  def testInitialize(self):
    self.kfloc.initialize(0.0,
                          1000,
                          0.,
                          0.,
                          0.,
                          0.,
                          0.,
                          0.,
                          0.,
                          0.,
                          0.)
    t = [ None ] * 1000
    t[-1] = [0.0, None]
    self.assertTrue( array_equiv(self.kfloc.buffer.data, t ))
    
    
    
if __name__ == '__main__':
  unittest.main()