import numpy as np
import math
from abc import ABCMeta
import logging

from BaseMethods import interp1d

class PointCloud:
  def __init__(self, N = 0):
    self.points = np.zeros( (3,N))
    
  def fromScan(self, scan, tt, xx, yy, hh):
    log = logging.getLogger('fromScan')
    
    scan_ = scan.copy()
    scan_.cleanUp()
    n = scan_.theta.shape[0]

    tt_ = scan_.tt
    xx_ = np.array( interp1d( tt_, list(tt), list(xx) ))
    yy_ = np.array( interp1d( tt_, list(tt), list(yy) ))
    hh_ = np.array( interp1d( tt_, list(tt), list(hh) ))

    self.points = np.zeros((3, n ))
    log.debug("tt_.shape %s" % str(tt_.shape))
    log.debug("xx_.shape %s" % str(xx_.shape))
    log.debug("yy.shape %s" % str(yy_.shape))
    log.debug("hh.shape %s" % str(hh_.shape))
    log.debug("scan.range.shape %s" % str(scan_.range.shape))
    log.debug("scan.theta.shape %s" % str(scan_.theta.shape))
    log.debug("points[0,0:].shape %s" % str(self.points[0,0:].shape))
    log.debug("points[1,0:].shape %s" % str(self.points[1,0:].shape))
    log.debug("points[2,0:].shape %s" % str(self.points[2,0:].shape))
    self.points[0,0:] = xx_ + scan_.range * np.cos(scan_.theta + hh_)
    self.points[1,0:] = yy_ + scan_.range * np.sin(scan_.theta + hh_)
    self.points[2,0:] = tt_
    
    