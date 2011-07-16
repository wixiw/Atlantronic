import sys
sys.path.append( "../../../../src/KFLocalizator/PyMockup" )
from numpy import *
from random import *
import matplotlib.pyplot as plt

from KalmanFilter import *

#time step of mobile movement
dt = 0.1
# Initialization of state matrices
X = array([[0.0], [0.0], [0.1], [0.1]])
P = diag((0.01, 0.01, 0.01, 0.01))
A = array([[1, 0, dt , 0], [0, 1, 0, dt], [0, 0, 1, 0], [0, 0, 0, 1]])
Q = eye(X.shape[0])
B = eye(X.shape[0])
U = zeros((X.shape[0],1))

# Measurement matrices
Y = array([[X[0,0] + abs(random.randn(1)[0])], [X[1,0] + abs(random.randn(1)[0])]])
H = array([[1, 0, 0, 0], [0, 1, 0, 0]])
R = eye(Y.shape[0])
# Number of iterations in Kalman Filter
N_iter = 50


tt = arange(0.0, N_iter * dt, dt)
XXpred = zeros(N_iter)
XXesti = zeros(N_iter)

# Applying the Kalman Filter
for i in arange(0, N_iter):
  (X, P) = kf_predict(X, P, A, Q, B, U)
  XXpred[i] = X[0,0]
  (X, P, K, IM, IS) = kf_update(X, P, Y, H, R)
  XXesti[i] = X[0,0]
  Y = array([[X[0,0] + abs(0.1 * random.randn(1)[0])],[X[1, 0] +  abs(0.1 * random.randn(1)[0])]])
  
# Plot
fig = plt.figure()
ax = fig.add_subplot(111)
plt.plot( tt, XXpred, '-b')
plt.plot( tt, XXesti, '-g')
plt.show()