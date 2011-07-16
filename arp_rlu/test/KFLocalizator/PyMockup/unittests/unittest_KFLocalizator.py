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
    
    
    
if __name__ == '__main__':
  unittest.main()