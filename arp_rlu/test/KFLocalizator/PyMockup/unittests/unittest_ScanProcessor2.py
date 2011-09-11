# coding=utf-8
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
    
  
    
if __name__ == '__main__':
    logging.basicConfig(level=logging.ERROR) #level=logging.DEBUG  
    unittest.main()