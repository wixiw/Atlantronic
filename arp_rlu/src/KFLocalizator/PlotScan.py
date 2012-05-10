# coding=utf-8
import sys
sys.path.append( "./PyMockup" )
import numpy as np
import math
import random
import logging
logging.basicConfig(level=logging.INFO)
log = logging.getLogger('main')

import matplotlib.pyplot as plt
from matplotlib.collections import PatchCollection
import matplotlib.patches as mpatches

from Scan import Scan
import BaseMethods


scan = Scan()
scan.load(sys.argv[1])

#===============================================================================
# Ploting
#===============================================================================
fig = plt.figure()
axe = fig.add_subplot(111, aspect='equal')

# scan (ray and impacts)
for i in range(len(scan.range)):
  if scan.range[-1-i] > 0.:
    xImpact = np.cos(scan.theta[-1-i]) * scan.range[-1-i]
    yImpact = np.sin(scan.theta[-1-i]) * scan.range[-1-i]
    axe.plot( [xImpact] , [yImpact], 'xb' )
    #axe.plot( [0., xImpact] , [0., yImpact], '--b' )
axe.plot( [0] , [0], 'ob' )
# borders
axe.plot( [0., np.cos(min(scan.theta))], 
          [0., np.sin(np.min(scan.theta))], '-m')
axe.plot( [0., np.cos(max(scan.theta))], 
         [0., np.sin(max(scan.theta))], '-m')

plt.show()