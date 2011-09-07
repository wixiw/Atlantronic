# coding=utf-8
import sys
sys.path.append( "../../../../src/KFLocalizator/PyMockup" )
import unittest
from numpy import *
import random
import math
import logging

from ransac import *

Ntests = 100

class TestRansac(unittest.TestCase):
  def setUp(self):
    pass
    
  def testEmpty(self):
    pc = PointCloud()
    
  
if __name__ == '__main__':
    logging.basicConfig(level=logging.ERROR)
#    logging.basicConfig(level=logging.DEBUG)
    unittest.main()