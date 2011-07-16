from numpy import *
import math
import random

from KalmanFilter import *
from ScanProcessor import *

class KFLocalizator:
  def __init__(self):
    self.odoVelBuf = []
    self.proc = ScanProcessor()
    
  def initialize(self):
    pass
  
  def newOdoVelocity(self):
    pass
  
  def newScan(self):
    pass
  
  def getEstimate(self):
    pass
  
