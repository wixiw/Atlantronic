# coding=utf-8
import numpy as np

#================================
# Paramètres du simu

simu_cfg = {
# Durée
"duration" : 0.301,   # en s
            
# Generation du mouvement
"maxLinAcc" : 0.,#7.,         # en m/s**2
"maxRotAcc" : 0.,#6. * np.pi,  # en rad/s**2
"maxLinVel" : 1.,         # en m/s
"maxRotVel" : 2. * np.pi,  # en rad/s
"maxXPos"   :  1.3,       # en m
"minXPos"   : -1.3,       # en m
"maxYPos"   :  0.8,       # en m
"minYPos"   : -0.8,       # en m

# Erreur sur la position initiale
"sigmaInitialPosition" : 0.005,   # en m
"sigmaInitialHeading" : 0.001,   # en rad

# LRFSimulator
"sigmaLRF" : 0., #0.01,   # en m

# Simu des odo
"percentSigmaTransOdoVelocity" : 0.03,  # en pourcent (1.0 is 100%)
"minSigmaTransOdoVelocity" : 0.001, #0.01    # en m/s
"percentSigmaRotOdoVelocity" : 0.03,  # en pourcent  (1.0 is 100%)
"minSigmaRotOdoVelocity" : 0.01, #0.1   # en rad/s
"odoTimeStep" : 0.01, # en s

# Débrayage des odo et/ou du scan pour le debug
"odoSimu" : True,
"lrfSimu" : True,
}

visu_cfg = {
"arrowInit" : True,
"ellipseInit" : True,

"arrowTrue" : True,

"arrowOdo" : False,
"ellipseOdo" : False,

"arrowLrf" : True,
"ellipseLrf" : True,
"arrowUpdateLrf" : False,
"ellipseUpdateLrf" : False,

"scan" : True,
"zoom" : True,
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
          
"givePerfectLRFMeasures" : False,  # pour le debug seulement
}


