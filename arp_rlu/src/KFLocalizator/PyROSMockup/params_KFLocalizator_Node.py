# coding=utf-8
import numpy as np

#================================
# Paramètres du simu

simu_cfg = {
# Simu d'odo virtuels
"frequency" : 100, # en Hz
"sigmaTransOdoVelocity" : 0.5,
"sigmaRotOdoVelocity" : 0.5 * np.pi,
}


visu_cfg = {
"arrowInit" : True,
"ellipseInit" : True,

"arrowTrue" : True,

"arrowOdo" : True,
"ellipseOdo" : True,

"arrowLrf" : True,
"ellipseLrf" : True,
"arrowUpdateLrf" : False,
"ellipseUpdateLrf" : False,

"scan" : False,
"zoom" : False,
"save" : False,
}

#============================
# Paramètres du KFLocalizator

kf_cfg = {
# Erreur sur la position initiale
"sigmaInitialPosition" : 0.01,   # en m
"sigmaInitialHeading" : 0.001,   # en rad

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
}


