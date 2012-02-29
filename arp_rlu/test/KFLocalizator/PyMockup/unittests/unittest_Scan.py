# coding=utf-8
import sys
sys.path.append( "../../../../src/KFLocalizator/PyMockup" )
import unittest
import numpy as np
import math
import random
import logging
logging.basicConfig(level=logging.INFO)
log = logging.getLogger('main')


from Scan import Scan
import BaseMethods

import LRFSimulator
from BaseClasses import Circle
from BaseClasses import Segment

class TestScan(unittest.TestCase):
  def setUp(self):
    pass
  
  def testExport(self):
    #===============================================================================
    # trajectory generation
    #===============================================================================
    ax = random.uniform( -6., 6.)
    ay = random.uniform( -6., 6.)
    ah = random.uniform( -4. * np.pi, 4. * np.pi)
    log.info("ax:%f",ax)
    log.info("ay:%f",ay)
    log.info("ah:%f",ah*180.0/np.pi)
    
    vx = random.uniform( -3., 3.)
    vy = random.uniform( -3., 3.)
    vh = random.uniform( -2. * np.pi, 2. * np.pi)
    log.info("vx:%f",vx)
    log.info("vy:%f",vy)
    log.info("vh:%f",vh*180.0/np.pi)
    
    x = random.uniform( -1.3, 1.3)
    y = random.uniform( -0.8, 0.8)
    h = random.uniform( 0., 2. * np.pi)
    
    tt = np.arange( 0.0, 0.1, 0.1 / 1024.)
    xx = tt**2 * ax + tt * vx + x
    yy = tt**2 * ay + tt * vy + y
    hh = tt**2 * ah + tt * vh + h
    
    xx = np.clip(xx, -1.3, 1.3)
    yy = np.clip(yy, -0.8, 0.8)
    
    
    lrfsim = LRFSimulator.LRFSimulator()
    lrfsim.sigma = 0.01
    
    
    #===============================================================================
    # add objects
    #===============================================================================
    nbObjects = 10
    lrfsim.objects = []
    for i in range(nbObjects):
      obj = Circle()
      obj.radius  = 0.04
      penetration = True
      while penetration:
        penetration = False
        obj.xCenter = random.uniform( -1.5, 1.5)
        obj.yCenter = random.uniform( -1.0, 1.0)
        for i in range(len(tt)):
          x_ = xx[i]
          y_ = yy[i]
          penetration = penetration or (np.linalg.norm( np.array( [ [obj.xCenter - x_], [obj.yCenter - y_] ] )) < obj.radius)
      lrfsim.objects.append(obj)
      
    log.info("Nb of objects :%d", len(lrfsim.objects))
    for o in lrfsim.objects:
      log.info("x:%f - y:%f - r:%f", o.xCenter, o.yCenter, o.radius)
      
      
    #===============================================================================
    # compute scan
    #===============================================================================
    scan = lrfsim.computeScan(tt, xx, yy, hh)
    
    scan.export("test.json")
    
    scan2 = Scan()
    scan2.load("test.json")
    
    self.assertTrue( len(scan.tt) == len(scan2.tt) )
    self.assertTrue( len(scan.range) == len(scan2.range) )
    self.assertTrue( len(scan.theta) == len(scan2.theta) )
    
    for i in range(len(scan.tt)):
      self.assertAlmostEqual( scan.tt[i], scan2.tt[i] )
      self.assertAlmostEqual( scan.range[i], scan2.range[i] )
      self.assertAlmostEqual( scan.theta[i], scan2.theta[i] )
    
  
if __name__ == '__main__':
#    logging.basicConfig(level=logging.ERROR) 
    logging.basicConfig(level=logging.DEBUG)  
    unittest.main()