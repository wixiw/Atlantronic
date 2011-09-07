# coding=utf-8
from numpy import *
import math
import random
import logging

from BaseClasses import PointCloud
from kmeans import *

class ClusterizeParams:
  def __init__(self):
    self.kmeansMaxIt = 10
    self.kmeansDispThreshold = 0.01
    self.minNbPoints = 5
    self.maxStddev = 0.1

def clusterize(pc, params):
  log = logging.getLogger('clusterize')
  log.debug("pc.points: %s", str(pc.points))
  
  n = pc.points.shape[1]
  if n == 0:
    log.warning("PointCloud is empty. Return (PointCloud(),PointCloud())")
    return []
  if n < params.minNbPoints:
    log.warning("Less than %d points in PointCloud." % params.minNbPoints)
    return []

  xMeans = ones((n)) * mean( pc.points[0,0:])
  yMeans = ones((n)) * mean( pc.points[1,0:])
  stddev = square(xMeans - pc.points[0,0:]) + square(yMeans - pc.points[1,0:])
  stddev = math.sqrt(sum(stddev) / n) 
  log.debug("Clusterize : %d points with stddev=%f (compared to %f)" % (n , stddev, params.maxStddev))
  
  if stddev < params.maxStddev:
    return [pc]
  
  (pcl, pcr) = kmeans(pc, params.kmeansMaxIt, params.kmeansDispThreshold)
  
  return clusterize(pcl, params) + clusterize(pcr, params)
