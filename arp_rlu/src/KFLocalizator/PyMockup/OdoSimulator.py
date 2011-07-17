from numpy import *
import random

from BaseClasses import OdoVelocity

class OdoSimulator:
  def __init__(self):
    self.sigma = 0.

  def computeOdoVelocity(self, dx, dy, da, t):
    # dx numpy array size (1,N) for d(x)/dt
    # dy numpy array size (1,N) for d(y)/dt
    # da numpy array size (1,N) for d(heading)/dt
    # t  numpy array size (1,N) for time
    
    if len(dx) != len(t):
      raise NameError('dx and t do not have same length')
    if len(dy) != len(t):
      raise NameError('dy and t do not have same length')
    if len(da) != len(t):
      raise NameError('da and t do not have same length')
    
    # vx is mean of dx
    # vy is mean of dy
    # va is mean of da
    ov = OdoVelocity()
    ov.vx = mean(dx) + random.normalvariate(0.0, self.sigma)
    ov.vy = mean(dy) + random.normalvariate(0.0, self.sigma)
    ov.va = mean(da) + random.normalvariate(0.0, self.sigma)
    return ov
    