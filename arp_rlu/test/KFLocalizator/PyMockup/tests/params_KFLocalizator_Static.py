# coding=utf-8
import numpy as np


#================================
# Paramètres du simu

# Erreur sur la position initiale
sigmaInitialPosition = 0.1   # en m
sigmaInitialHeading = 0.2   # en rad

# LRFSimulator
sigmaLRF = 0.01   # en m


#============================
# Paramètres du KFLocalizator

# Performance de l'odométrie
sigmaTransOdoVelocity = 0.001    # en m
sigmaRotOdoVelocity = 0.001   # en rad

# Performance du LRF
sigmaLaserRange = 0.01    # en m
sigmaLaserAngle = 0.001    # en rad
sigmaSegmentHeading = 0.5   # en rad

# relatif au IEKF
Nit = 10
threshold = np.array([ [0.005], [0.005], [0.01]])