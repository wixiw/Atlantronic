# coding=utf-8
import numpy as np
import logging

# inputs : 
# X : The mean state estimate of the previous step ( k -1 ).
# P : The state covariance of previous step ( k -1 ).
# A : The transition n x n matrix.
# Q: The process noise covariance matrix.
# B : The input effect matrix.
# U : The control input.
# outputs :
# X : The predicted mean state estimate of current step ( k )
# P : The predicted state covariance of current step ( k )
def kf_predict(X, P, A, Q, B, U):
  log = logging.getLogger('kf_predict')
  X = np.dot(A, X) + np.dot(B, U)
  P = np.dot(A, np.dot(P, A.T)) + Q
  return(X,P)
  
# inputs :
# X : The predicted mean state estimate of current step ( k )
# P : The predicted state covariance of current step ( k )
# Y : The measurement of current step ( k )
# H : measurement model such as : Y = H.X + N(0,R)
# R : measurement noise covariance matrix
# outputs :
# X : The mean state estimate of the current step ( k ).
# P : The state covariance of current step ( k ).
# K : the Kalman Gain matrix
# IM : the Mean of predictive distribution of Y
# IS : the Covariance or predictive mean of Y
def kf_update(X, P, Y, H, R):
  log = logging.getLogger('kf_update')
  IM = np.dot(H, X)
  IS = R + np.dot(H, np.dot(P, H.T))
  K = np.dot(P, np.dot(H.T, np.linalg.inv(IS)))
  X = X + np.dot(K, (Y-IM))
  P = P - np.dot(K, np.dot(IS, K.T))
  return (X,P,K,IM,IS)
  
# inputs :
# X : The predicted mean state estimate of current step ( k )
# P : The predicted state covariance of current step ( k )
# Y : The measurement of current step ( k )
# H : measurement model such as : Y = H.X + N(0,R)
# R : measurement noise covariance matrix
# IM : the Mean of predictive distribution of Y
# outputs :
# X : The mean state estimate of the current step ( k ).
# P : The state covariance of current step ( k ).
# K : the Kalman Gain matrix
# IM : the Mean of predictive distribution of Y
# IS : the Covariance or predictive mean of Y
def ekf_update(X, P, Y, H, R, IM):
  log = logging.getLogger('ekf_update')
  IS = R + np.dot(H, np.dot(P, H.T))
  K = np.dot(P, np.dot(H.T, np.linalg.inv(IS)))
  X = X + np.dot(K, (Y-IM))
  P = P - np.dot(K, np.dot(IS, K.T))
  return (X,P,K,IM,IS)
  
# inputs :
# X : The predicted mean state estimate of current step ( k )
# P : The predicted state covariance of current step ( k )
# Y : The measurement of current step ( k )
# J : jacobian of measurement model such as : Y = J(X).X + N(0,R)
# R : measurement noise covariance matrix
# IM : the Mean of predictive distribution of Y
# Nit : max nb of iteration
# thrershold : stop if abs(X[k]|i] - X[k-1][i]) < threshold[i] for all i
# outputs :
# X : The mean state estimate of the current step ( k ).
# P : The state covariance of current step ( k ).
# K : the Kalman Gain matrix
# IM : the Mean of predictive distribution of Y
# IS : the Covariance or predictive mean of Y
# k  : nb of iterations computed
def iekf_update(X, P, Y, J, R, IM, Nit, threshold):
  log = logging.getLogger('iekf_update')
  X_ = X
  P_ = P
  IM_ = IM
  for k in range(Nit):
    X__ = X_
    H = J(X_)
    (X_,P_,K,IM,IS) = ekf_update(X, P, Y, H, R, IM)
    if np.all(np.less_equal( np.abs(X_ - X__), threshold ) ):
      break
  log.debug("Nit: %d", k+1)
  return (X_,P_,K,IM,IS,k+1)
      