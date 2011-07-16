import sys
sys.path.append( "../../../../src/KFLocalizator/PyMockup" )
import unittest
from numpy import *
import random
import math

from ScanProcessor import *

class TestScanProcessor(unittest.TestCase):
  def setUp(self):
    self.scanproc = ScanProcessor()
    
  def testInit(self):
    self.assertEqual(self.scanproc.beacons, [])
    
  def testFindClusters(self):
    xx = []
    yy = []
    aa = []
    scan = Scan()
    self.scanproc.findCluster(xx, yy, aa)
    self.assertEqual(self.scanproc.objects, [])

if __name__ == '__main__':
    unittest.main()