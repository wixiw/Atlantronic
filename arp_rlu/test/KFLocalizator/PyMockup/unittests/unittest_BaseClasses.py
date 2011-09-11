# coding=utf-8
import sys
sys.path.append( "../../../../src/KFLocalizator/PyMockup" )
import unittest
import numpy as np
import math
import random
import logging

from BaseClasses import *

class TestBaseClasses(unittest.TestCase):
  def setUp(self):
    pass

  def testComputePointCloud1(self):
    scan = Scan()
    scan.tt    = np.arange(0., 1., 1./4. )
    scan.theta = np.zeros( (4) )
    scan.range = np.zeros( (4) )
    scan.check()
    tt = np.arange(0., 1., 1./4. )
    xx = np.zeros( (4) )
    yy = np.zeros( (4) )
    hh = np.zeros( (4) )
    pc = PointCloud()
    pc.fromScan(scan, tt, xx, yy, hh)
    print "pc.points"
    print pc.points
    self.assertTrue(np.array_equiv( pc.points, np.array( [[ 0. , 0.  , 0.   , 0.  ], 
                                                          [ 0. , 0.  , 0.   , 0.  ],
                                                          [ 0. , 0.25, 0.50 , 0.75 ]]) ))
    
  def testComputePointCloud2(self):
    scan = Scan()
    scan.tt    = np.arange(0., 1., 1./4. )
    scan.theta = np.zeros( (4) )
    scan.theta[0:] = [0., np.pi/2., np.pi, -np.pi/2.]
    scan.range = np.zeros( (4) )
    scan.range[0:] = [1.0, 0.5, 2.0, 3.0]
    scan.check()
    tt = np.arange(0., 1., 1./2. )
    xx = np.ones( (2) ) * 0.3
    yy = np.ones( (2) ) * -0.1
    hh = np.ones( (2) ) * np.pi/2.
    pc = PointCloud()
    pc.fromScan(scan, tt, xx, yy, hh)
    print "pc.points"
    print pc.points
    self.assertTrue(np.allclose( pc.points, np.array( [[ 0.3, -0.2, 0.3  , 3.3  ], 
                                                       [ 0.9, -0.1,-2.1  ,-0.1  ],
                                                       [ 0. , 0.25, 0.50 , 0.75 ]]) ))

    
if __name__ == '__main__':
##    logging.basicConfig(level=logging.ERROR) 
    logging.basicConfig(level=logging.DEBUG) 
    unittest.main()