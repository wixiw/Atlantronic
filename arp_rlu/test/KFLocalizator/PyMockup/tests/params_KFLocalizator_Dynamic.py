# coding=utf-8
import numpy as np

#================================
# Paramètres du simu

simu_cfg = {
# Durée
"duration" : 0.601,   # en s
            
# Espace de naviguation
"maxXPos"   :  1.3,       # en m
"minXPos"   : -1.3,       # en m
"maxYPos"   :  0.8,       # en m
"minYPos"   : -0.8,       # en m

# Generation d'un mouvement méchant
"maxLinAcc" : 7.,         # en m/s**2
"maxRotAcc" : 6. * np.pi,  # en rad/s**2
"maxLinVel" : 3.,         # en m/s
"maxRotVel" : 2. * np.pi,  # en rad/s

## Generation d'un mouvement gentil
#"maxLinAcc" : 0.,         # en m/s**2
#"maxRotAcc" : 0.,  # en rad/s**2
#"maxLinVel" : 0.5,         # en m/s
#"maxRotVel" : 0.5 * np.pi,  # en rad/s

# Erreur sur la position initiale
"sigmaInitialPosition" : 0.005,   # en m
"sigmaInitialHeading" : 0.001,   # en rad

# LRFSimulator
"sigmaLRF" : 0.01,   # en m

# Simu des odo
"percentSigmaTransOdoVelocity" : 0.03,  # en pourcent (1.0 is 100%)
"minSigmaTransOdoVelocity" : 0.001, #0.01    # en m/s
"percentSigmaRotOdoVelocity" : 0.03,  # en pourcent  (1.0 is 100%)
"minSigmaRotOdoVelocity" : 0.01, #0.1   # en rad/s
"odoTimeStep" : 0.01, # en s

# Simu d'odo virtuels
"virtualOdo" : False,
"virtualSigmaTransOdoVelocity" : 0.5,
"virtualSigmaRotOdoVelocity" : 0.5 * np.pi,

# Débrayage des odo et/ou du scan pour le debug
"odoSimu" : True,
"lrfSimu" : True,
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
"sigmaInitialPosition" : 0.005,   # en m
"sigmaInitialHeading" : 0.001,   # en rad

# Performance de l'odométrie (initiale seulement)
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


