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


  

