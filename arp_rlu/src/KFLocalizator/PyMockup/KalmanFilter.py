from numpy import *

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
  X = dot(A, X) + dot(B, U)
  P = dot(A, dot(P, A.T)) + Q
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
  IM = dot(H, X)
  IS = R + dot(H, dot(P, H.T))
  K = dot(P, dot(H.T, linalg.inv(IS)))
  X = X + dot(K, (Y-IM))
  P = P - dot(K, dot(IS, K.T))
  return (X,P,K,IM,IS)
