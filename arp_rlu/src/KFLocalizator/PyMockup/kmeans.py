# coding=utf-8
from numpy import *
import math
import random
import logging

from BaseClasses import PointCloud

def kmeans(pc, maxIt, dispTh):
    log = logging.getLogger('kmeans')
    log.debug("pc.points: %s", str(pc.points))
    
    n = pc.points.shape[1]
    if n == 0:
      log.warning("PointCloud is empty. Return (PointCloud(),PointCloud())")
      return (PointCloud(),PointCloud())
    if n == 1:
      log.warning("Only one point in PointCloud. Return (pc,PointCloud())")
      return (pc,PointCloud())
    
    idFirst = random.randint(0, n-1)
    idSecond = idFirst
    while idSecond == idFirst:
      idSecond = random.randint(0, n-1)
      
    
    log.debug("idFirst: %d - idSecond: %d", idFirst, idSecond)
    xFirst = pc.points[0,idFirst]
    yFirst = pc.points[1,idFirst]
    xSecond = pc.points[0,idSecond]
    ySecond = pc.points[1,idSecond]
    log.debug("xFirst: %f - yFirst: %f", xFirst, yFirst)
    log.debug("xSecond: %f - ySecond: %f", xSecond, ySecond)
    
    xxFirst = ones( (n) ) * xFirst
    yyFirst = ones( (n) ) * yFirst
    xxSecond = ones( (n) ) * xSecond
    yySecond = ones( (n) ) * ySecond
    log.debug("xxFirst: %s", str(xxFirst))
    log.debug("yyFirst: %s", str(yyFirst))
    log.debug("xxSecond: %s", str(xxSecond))
    log.debug("yySecond: %s", str(yySecond))
    
    nbIt = 0
    while nbIt < maxIt:
      nbIt = nbIt + 1
      log.debug("*********** nbIt: %d ***********", nbIt)
      
      distFirst = sqrt( square(xxFirst - pc.points[0,0:]) + square(yyFirst - pc.points[1,0:]) )
      distSecond = sqrt( square(xxSecond - pc.points[0,0:]) + square(yySecond - pc.points[1,0:]) )
      log.debug("distFirst: %s", str(distFirst))
      log.debug("distSecond: %s", str(distSecond))
      
      first = less_equal( distFirst, distSecond )
      second = logical_not(first)
      log.debug("first: %s", str(first))
      log.debug("second: %s", str(second))
      
      xFirst = mean( pc.points[0,nonzero(first)] )
      yFirst = mean( pc.points[1,nonzero(first)] )
      xSecond = mean( pc.points[0,nonzero(second)] )
      ySecond = mean( pc.points[1,nonzero(second)] )
      log.debug("xFirst: %f - yFirst: %f", xFirst, yFirst)
      log.debug("xSecond: %f - ySecond: %f", xSecond, ySecond)
      
      distFirst = math.sqrt( (xxFirst[0]-xFirst)**2 + (yyFirst[0]-yFirst)**2 )
      distSecond = math.sqrt( (xxSecond[0]-xSecond)**2 + (yySecond[0]-ySecond)**2 )
      log.debug("distFirst: %f", distFirst)
      log.debug("distSecond: %f", distSecond)
      
      xxFirst = ones( (n) ) * xFirst
      yyFirst = ones( (n) ) * yFirst
      xxSecond = ones( (n) ) * xSecond
      yySecond = ones( (n) ) * ySecond
      
      if distFirst < dispTh and distSecond < dispTh:
        log.info("Break on displacement threshold (%f)", dispTh)
        break
    if nbIt == maxIt:
      log.warning("Convergence failed => max iteration (%d) reached !", maxIt)
      return (pc,PointCloud())
    
    pcFirst = PointCloud()
    pcFirst.points = reshape(pc.points[0:, nonzero(first)], (2,-1))
    pcFirst.tt = squeeze(pc.tt[nonzero(first)])
    pcSecond = PointCloud()
    pcSecond.points = reshape(pc.points[0:, nonzero(second)], (2,-1))
    pcSecond.tt = squeeze(pc.tt[nonzero(second)])
    log.debug("pcFirst.points: %s", str(pcFirst.points))
    log.debug("pcSecond.points: %s", str(pcSecond.points))
    return (pcFirst, pcSecond)
