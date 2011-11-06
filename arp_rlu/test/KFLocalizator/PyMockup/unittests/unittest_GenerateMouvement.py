# coding=utf-8
import sys
sys.path.append( "../../../../src/KFLocalizator/PyMockup" )
import unittest
import numpy as np
import random
import math
import logging

import GenerateMouvement
from BaseClasses import *


Ntests = 2

class TestGenerateMouvement(unittest.TestCase):
  def setUp(self):
    pass
  
  def testStatic(self):
    dt = 0.1/1024.
    params = {
              "duration"  : 1.,   # en s
              "maxLinAcc" : 0.,   # en m/s**2
              "maxRotAcc" : 0.,   # en rad/s**2
              "maxLinVel" : 0.,   # en m/s
              "maxRotVel" : 0.,   # en rad/s
              "maxXPos"   :  1.3, # en m
              "minXPos"   : -1.3, # en m
              "maxYPos"   :  0.8, # en m
              "minYPos"   : -0.8, # en m
              }
    for k in range(Ntests):
      tt,xx,yy,hh,vx,vy,vh,ax,ay,ah = GenerateMouvement.gen(params)
      self.assertEqual( len(tt), params["duration"] / dt )
      for i in range(1, len(tt)):
        self.assertAlmostEqual( tt[i] - tt[i-1], 0.1/1024. )
      self.assertEqual( len(tt), len(xx) )
      self.assertEqual( len(tt), len(yy) )
      self.assertEqual( len(tt), len(hh) )
      self.assertEqual( len(tt), len(vx) )
      self.assertEqual( len(tt), len(vy) )
      self.assertEqual( len(tt), len(vh) )
      self.assertEqual( len(tt), len(ax) )
      self.assertEqual( len(tt), len(ay) )
      self.assertEqual( len(tt), len(ah) )
      self.assertTrue(np.array_equiv( xx , np.ones_like(tt) * xx[0] ))
      self.assertTrue(np.array_equiv( yy , np.ones_like(tt) * yy[0] ))
      self.assertTrue(np.array_equiv( hh , np.ones_like(tt) * hh[0] ))
      self.assertTrue(np.array_equiv( vx , np.ones_like(tt) * vx[0] ))
      self.assertTrue(np.array_equiv( vy , np.ones_like(tt) * vy[0] ))
      self.assertTrue(np.array_equiv( vh , np.ones_like(tt) * vh[0] ))
      self.assertTrue(np.array_equiv( ax , np.ones_like(tt) * ax[0] ))
      self.assertTrue(np.array_equiv( ay , np.ones_like(tt) * ay[0] ))
      self.assertTrue(np.array_equiv( ah , np.ones_like(tt) * ah[0] ))
    
  def testDynamic(self):
    log = logging.getLogger('testDynamic')
    dt = 0.1/1024.
    params = {
              "duration"  : 1.,         # en s
              "maxLinAcc" : 6.,         # en m/s**2  
              "maxRotAcc" : 4.* np.pi,  # en rad/s**2
              "maxLinVel" : 3.,         # en m/s     
              "maxRotVel" : 2 * np.pi,  # en rad/s   
              "maxXPos"   :  1.3,       # en m       
              "minXPos"   : -1.3,       # en m       
              "maxYPos"   :  0.8,       # en m       
              "minYPos"   : -0.8,       # en m       
              }
    for k in range(Ntests):
      tt,xx,yy,hh,vx,vy,vh,ax,ay,ah = GenerateMouvement.gen(params)
      
      log.debug("tt: \n%s", repr(tt))
      log.debug("xx: \n%s", repr(xx))
      log.debug("yy: \n%s", repr(yy))
      log.debug("hh: \n%s", repr(hh))
      log.debug("vx: \n%s", repr(vx))
      log.debug("vy: \n%s", repr(vy))
      log.debug("vh: \n%s", repr(vh))
      log.debug("ax: \n%s", repr(ax))
      log.debug("ay: \n%s", repr(ay))
      log.debug("ah: \n%s", repr(ah))
      
      self.assertEqual( len(tt), params["duration"] / dt )
      self.assertEqual( len(tt), len(xx) )
      self.assertEqual( len(tt), len(yy) )
      self.assertEqual( len(tt), len(hh) )
      self.assertEqual( len(tt), len(vx) )
      self.assertEqual( len(tt), len(vy) )
      self.assertEqual( len(tt), len(vh) )
      self.assertEqual( len(tt), len(ax) )
      self.assertEqual( len(tt), len(ay) )
      self.assertEqual( len(tt), len(ah) )
      
      for i in range(1, len(tt)):
        self.assertAlmostEqual( tt[i] - tt[i-1], dt )
        
        self.assertAlmostEqual( xx[i], xx[i-1] + vx[i-1] * dt, places=3 )
        self.assertAlmostEqual( yy[i], yy[i-1] + vy[i-1] * dt, places=3 )
        self.assertAlmostEqual( betweenMinusPiAndPlusPi(hh[i]), betweenMinusPiAndPlusPi(hh[i-1] + vh[i-1] * dt), places=3 )
        
        self.assertAlmostEqual( vx[i], vx[i-1] + ax[i-1] * dt, places=3 )
        self.assertAlmostEqual( vy[i], vy[i-1] + ay[i-1] * dt, places=3 )
        self.assertAlmostEqual( betweenMinusPiAndPlusPi(vh[i]), betweenMinusPiAndPlusPi(vh[i-1] + ah[i-1] * dt), places=3 )
        
        
#      self.assertTrue( np.max(xx) <= params["maxXPos"] )
#      self.assertTrue( np.min(xx) >= params["minXPos"] )
#      self.assertTrue( np.max(yy) <= params["maxYPos"] )
#      self.assertTrue( np.min(yy) >= params["minYPos"] )
#      
#      self.assertTrue( np.max(vx) <=  params["maxLinVel"] )
#      self.assertTrue( np.min(vx) >= -params["maxLinVel"] )
#      self.assertTrue( np.max(vy) <=  params["maxLinVel"] )
#      self.assertTrue( np.min(vy) >= -params["maxLinVel"] )
#      self.assertTrue( np.max(vh) <=  params["maxRotVel"] )
#      self.assertTrue( np.min(vh) >= -params["maxRotVel"] )
#      
#      self.assertTrue( np.max(ax) <=  params["maxLinAcc"] )
#      self.assertTrue( np.min(ax) >= -params["maxLinAcc"] )
#      self.assertTrue( np.max(ay) <=  params["maxLinAcc"] )
#      self.assertTrue( np.min(ay) >= -params["maxLinAcc"] )
#      self.assertTrue( np.max(ah) <=  params["maxRotAcc"] )
#      self.assertTrue( np.min(ah) >= -params["maxRotAcc"] )
    
    
  
if __name__ == '__main__':
    logging.basicConfig(level=logging.ERROR)
#    logging.basicConfig(level=logging.DEBUG)
    unittest.main()