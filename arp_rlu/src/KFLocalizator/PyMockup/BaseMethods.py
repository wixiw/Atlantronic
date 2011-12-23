import numpy as np
import math
import logging


def betweenMinusPiAndPlusPi(angle):
  angle = betweenZeroAndTwoPi( angle )
  if angle > np.pi:
    angle = angle - 2 * np.pi
  if angle < -np.pi:
    angle = angle + 2 * np.pi
  return angle

def betweenZeroAndTwoPi(angle):
  return np.fmod( np.fmod(angle, 2 * np.pi) + 4. * np.pi, 2. * np.pi);


def interp1d( t, tp, yp ):
    y = np.interp(t, tp, yp)
    if len(tp) < 2:
        return y
    for i in range(len(t)):
        if t[i] < tp[0]:
            y[i] = yp[0] - (tp[0]-t[i])*(yp[1]-yp[0])/(tp[1]-tp[0])
        if t[i] > tp[-1]:
            y[i] = yp[-1] + (t[i]-tp[-1])*(yp[-1]-yp[-2])/(tp[-1]-tp[-2])
    return y

def interpMatrix( t, tp, yp):
  y = []
  for k in range(len(t)):
    y.append( np.zeros( yp[0].shape ) )
  for i in range(int(yp[0].shape[0])):
    for j in range(int(yp[0].shape[1])):
      yp_ = []
      for k in range(len(tp)):
        yp_.append(yp[k][i,j])
      for k in range(len(t)):
        y[k][i,j] = interp1d([t[k]], tp, yp_)
  return y

def pca(x):
  means = np.mean(x, 1)
  cov = np.cov( x - means.reshape((2,1)) ) 
  values, vectors = np.linalg.eig( cov )
  return means, np.sqrt(values), vectors

def getEllipseParametersFromEstimate(estim):
  if estim.covariance is None:
    xy = (estim.xRobot, estim.yRobot)
    return xy, 0., 0., 0.
  cov = estim.covariance[0:2, 0:2]
  values, vectors = np.linalg.eig( cov )
  stddev = np.sqrt(values)
  axe = vectors[0:, np.argmax(stddev)]
  height = 3 * np.min(stddev)
  width = 3 * np.max(stddev)
  xy = (estim.xRobot, estim.yRobot)
  angle = math.atan2( axe[1], axe[0]) * 180. / np.pi
  return xy, width, height, angle


def getMedian(v):
  if len(v) == 1:
    return v[0]
  noChange = False
  while not noChange:
    noChange = True
    for j in range(len(v)-1):
      if v[j] > v[j+1]:
        tmp = v[j+1]
        v[j+1] = v[j]
        v[j] = tmp
        noChange = False
  return v[(len(v)-1)/2]

def enumerateCombinations(x, n):
  out = []
  if n < 1:
    return out
  if n == 1:
    return [ [v] for v in x]
  m = len(x)
  if m < n:
    return out
  if m == n:
    return [ x ]
  
  for i in range(m-n+1):
    combs = enumerateCombinations(x[i+1:], n-1)
    for c in combs:
      d = [v for v in c]
      d.insert(0,x[i])
      out.append( d )
  return out

def enumeratePermutations(x):
  m = len(x)
  if m == 1:
    return [x]
  if m == 2:
    return [ [x[0], x[1]], [x[1], x[0]] ]
  if m < 0:
    return []
  
  out = []
  
  for i in range(m):
    x_ = [v for v in x] 
    x_.pop(i)
    combs = enumeratePermutations(x_)
    for c in combs:
      d = [v for v in c]
      d.insert(0,x[i])
      out.append( d )
  return out

