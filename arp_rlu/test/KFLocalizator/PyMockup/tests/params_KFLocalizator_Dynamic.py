# coding=utf-8
import numpy as np

#================================
# Paramètres du simu

simu_cfg = {
# Durée
"duration" : 1.,  # en s
            
# Generation du mouvement
"maxLinAcc" : 6.,         # en m/s**2
"maxRotAcc" : 4 * np.pi,  # en rad/s**2
"maxLinVel" : 3.,         # en m/s
"maxRotVel" : 2 * np.pi,  # en rad/s
"maxXPos"   :  1.3,       # en m
"minXPos"   : -1.3,       # en m
"maxYPos"   :  0.8,       # en m
"minYPos"   : -0.8,       # en m

# Erreur sur la position initiale
"sigmaInitialPosition" : 0.05,   # en m
"sigmaInitialHeading" : 0.1,   # en rad

# LRFSimulator
"sigmaLRF" : 0.01,   # en m

# Simu des odo
"sigmaTransOdoVelocity" : 0.001, #0.01    # en m/s
"sigmaRotOdoVelocity" : 0.01, #0.1   # en rad/s
"odoTimeStep" : 0.01, # en s

"Nscans" : 5
}

visu_cfg = {
"ellipse" : True,
"intermediary_arrow" : True,
"intermediary_ellipse" : False,
"zoom" : True,
}


#============================
# Paramètres du KFLocalizator

kf_cfg = {
# Performance de l'odométrie
"sigmaTransOdoVelocity" : 0.01, #0.01    # en m/s
"sigmaRotOdoVelocity" : 0.01, #0.1   # en rad/s

# Performance du LRF
"sigmaLaserRange" : 0.005,    # en m
"sigmaLaserAngle" : 0.05,    # en rad
"sigmaSegmentHeading" : 0.5,   # en rad

# relatif au IEKF
"iekf_cfg" : {
              "Nit" : 10,
              "threshold" : np.array([ [0.005], [0.005], [0.01]]),
              },

# relatif au ScanProcessor
"scanproc_cfg" : { 
                  "maxDistance" : 0.9,      # erreur cartésienne max sur l'estimée de la position balise. En m
                  "thresholdRange" : 0.08,  # seuil de découpe de cluster sur r. En m
                  },

"givePerfectLRFMeasures" : False,  # for debug only
}


