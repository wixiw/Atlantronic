#!/usr/local/bin/python
# -*- coding: iso-8859-15 -*-

import sys, os, random
import math
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.collections import PatchCollection
import matplotlib.patches as mpatches

import scipy.stats
#import progressbar as pg
import pickle
#import IPython

#shell = IPython.Shell.IPShellEmbed()

def betweenZeroAndTwoPi(angle):
  return np.fmod( np.fmod(angle, 2 * np.pi) + 4. * np.pi, 2. * np.pi);

def betweenMinusPiAndPlusPi(angle):
  angle = betweenZeroAndTwoPi( angle )
  if angle > np.pi:
    angle = angle - 2 * np.pi
  if angle < -np.pi:
    angle = angle + 2 * np.pi
  return angle

generate = True

xMax = 1200
yMax = 950
xyStep = 20
thetaStep = 0.005
beacons = np.array( [[ 1500., -1500., -1500.  ],[ 0., 1050., -1050. ]] )
delta = 120. * np.pi / 180.

yy, xx = np.mgrid[-yMax:yMax+xyStep:xyStep, -xMax:xMax+xyStep:xyStep]

if generate is True:
  hh = np.arange(0., 2.*np.pi, thetaStep)
  
  data = []
  visibility = []
  for j in range(xx.shape[0]):
    data_ = []
    visibility_ = []
    for i in range(xx.shape[1]):
      data_.append([])
      visibility_.append([])
    data.append(data_)
    visibility.append(visibility_)
  width = np.zeros( (xx.shape[0], xx.shape[1]) )
  #widgets = [pg.Percentage(), ' ', pg.Bar(marker=pg.RotatingMarker())]
  #pbar = pg.ProgressBar(widgets=widgets, maxval=xx.shape[0]*xx.shape[1]).start()
  for i in range(xx.shape[1]):
    for j in range(xx.shape[0]):
      #pbar.update(i*xx.shape[0] + j + 1)
      theta = np.zeros( (3) )
      for b in range(3):
        theta[b] = np.arctan2(beacons[1,b]-yy[j,i], beacons[0,b]-xx[j,i])
      data[j][i] = np.zeros( (2,len(hh)) )
      for k in range(len(hh)):
        data[j][i][0,k] = hh[k]
        for b in range(3):
          if math.fabs(betweenMinusPiAndPlusPi(hh[k] - theta[b])) < delta:
            data[j][i][1,k] += 1 
      transitions = filter(lambda k:data[j][i][1,k] != data[j][i][1,k+1], range(len(hh)-1))
      if data[j][i][1, len(hh)-1] != data[j][i][1,0]:
        transitions.append(len(hh)-1)
      visibility[j][i] = []
      for t in range(len(transitions)):
        visibility[j][i].append( ( data[j][i][1,transitions[t]], betweenZeroAndTwoPi(data[j][i][0,transitions[t]] - data[j][i][0,transitions[t-1]]), data[j][i][0,transitions[t-1]], data[j][i][0,transitions[t]] ) )
      for v in visibility[j][i]:
        if v[0] < 2.:
          width[j,i] += v[1]
                 
  output = open("./width.dat", mode='w')
  pickle.dump(width, output)
  output.close()
  output = open("./visibility.dat", mode='w')
  pickle.dump(visibility, output)
  output.close()
else:
  file = open("width.dat", 'r')
  width = pickle.load(file)
  file = open("visibility.dat", 'r')
  visibility = pickle.load(file)


extent = (-xMax, xMax, -yMax, yMax)
levels = np.arange(0, 141., 10.)

fig = plt.figure(figsize=(15,10))
ax = fig.add_subplot(111, aspect="equal")
im = ax.imshow(width * 180. / np.pi, extent=extent, interpolation="bilinear")

cset = ax.contour(xx, yy, width * 180. / np.pi, levels, colors="k")
ax.clabel(cset, fontsize=11, inline=1)

for i in np.arange(5, xx.shape[1], 10):
  for j in np.arange(5, xx.shape[0], 10):
    for v in visibility[j][i]:
      if v[0] < 2.:
        angle1 = betweenZeroAndTwoPi(v[2]) * 180. / np.pi
        angle2 = betweenZeroAndTwoPi(v[3]) * 180. / np.pi
        wedge = mpatches.Wedge([xx[j,i],yy[j,i]], 50, angle1, angle2, ec="k", fc="w")
        ax.add_patch(wedge)

cbar = plt.colorbar(im)
cbar.set_label("Largeur en deg de la plage d'angle ou on voit qu'une seule balise")
ax.set_title("Carte de visibilite balise pour un LRF de 240 deg")
ax.set_xlabel("x (mm)")
ax.set_ylabel("y (mm)")

plt.show()
