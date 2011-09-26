# coding=utf-8
import sys
from mpl_toolkits.mplot3d import Axes3D
from numpy import *
from random import *
import matplotlib.pyplot as plt
from pylab import *
import pdb
from math import *

n_discret=100
Xactuel = array([0,0,0])
Xfinal=array([1.000,0,0])
vmax=1.0
accmax=1.5

#discretisation de la trajectoire
def interpolate(i): 
    return Xactuel+(Xfinal-Xactuel)/n_discret*i

#la le range va de 0 à 99
#donc j'ai Xactuel mais pas Xfinal
Xtraj=map(interpolate, range(n_discret))

#je rajoute le debut et la fin pour avoir les conditions aux limites
Xtraj=concatenate(([Xactuel],Xtraj,[Xfinal],[Xfinal]))

#attention maintenant que j'ai fait ca, j'ai
# 0: Xactuel
#1: Xactuel
#2: premier pas de traj
# 100: dernier pas de traj
# 101 et 102: Xfinal

#calcul vitesse, acc du NP 
def Xip(Xi_m1,Xi,Xi_p1,dti):
    Xi_m12=(Xi_m1+Xi)/2
    Xi_p12=(Xi+Xi_p1)/2
    return (Xi_p12-Xi_m12)/dti

def Xipp(Xi_m1,Xi,Xi_p1,dti_m1,dti,dti_p1):
    Xip_m12=(Xi-Xi_m1)/(dti/2+dti_m1/2)
    Xip_p12=(Xi_p1-Xi)/(dti_p1/2+dti/2)
    #pdb.set_trace()  
    return (Xip_p12-Xip_m12)/dti
    
#creation de la contrainte vmax    
def contrainte_vmax(i,dt_test):
    vect_vitesse=Xip(Xtraj[i-1],Xtraj[i],Xtraj[i+1],dt_test)
    vitesse=sqrt(vect_vitesse[0]*vect_vitesse[0]+vect_vitesse[1]*vect_vitesse[1])
    return vmax-vitesse

def getspeed(i):
    vect_vitesse=Xip(Xtraj[i-1],Xtraj[i],Xtraj[i+1],dt[i])
    vitesse=sqrt(vect_vitesse[0]*vect_vitesse[0]+vect_vitesse[1]*vect_vitesse[1])
    return vitesse

#creation de la contrainte vmax    
def contrainte_accmax(i,dt_test):
    vect_acc=Xipp(Xtraj[i-1],Xtraj[i],Xtraj[i+1],dt[i-1],dt_test,dt[i+1])
    acc=sqrt(vect_acc[0]*vect_acc[0]+vect_acc[1]*vect_acc[1])
    return accmax-acc

def getCriteria(value):
    if value>0:
        return value
    else:
        return value*value

def criteria(val,i):
    return getCriteria(contrainte_vmax(i,val))+getCriteria(contrainte_accmax(i,val))

def diff(fonction, val):
    epsilon=0.0001
    return (fonction(val+epsilon)-fonction(val))/epsilon
        
def findMinimum(fonction,mini,maxi):
    t=mini
    stp=(maxi-mini)/2.0
    while (stp>0.0001):
        #pdb.set_trace()
        if diff(fonction,t)<0:
            t=t+stp
        else:
            t=t-stp
        stp=stp/2.0
    return t    
            
        
    


#initialisation des dt
dt=ones(n_discret+3)*0.2
newdt=copy(dt)
notDone=True

nturn=0
nmaxturn=50

dt_evol=copy([dt[:]])
print "debut"
 


while notDone:
    if mod(nturn,10)==0:
        print "tour d'optim %i"%nturn
#    if nturn==4:
#         pdb.set_trace()
#         axspeed = plt.figure().add_subplot(111)
#         xdt=array(range(1000))
#         xdt=(xdt)/2000.0+0.008
#        fonction=lambda t:criteria(t,3)
#        ydt=map(fonction,xdt)
#        plt.plot(xdt,ydt)      
#        plt.show() 
        
    for i in range(2,n_discret+1):
        fonction=lambda t:criteria(t,i)
        newdt[i]=findMinimum(fonction,0.0001,10)
      
    dt[2:n_discret+1]=copy(newdt[2:n_discret+1])
    dt_evol=concatenate((dt_evol,copy([dt[:]])))
    if sum(newdt-dt)>0.1 or nturn==nmaxturn-1:
        notDone=False
    
#    if nturn==0 or nturn==1 or nturn==2 or nturn==3:
#        axspeed = plt.figure().add_subplot(111)
#        xspeed=range(1,n_discret+2)
#        yspeed=map(getspeed,xspeed)
#        plt.plot(xspeed,yspeed)
        
    nturn=nturn+1
    
#plt.show()    
        
print "termine"
   
 
        
fig = plt.figure()
ax = Axes3D(fig)
X=range(n_discret+3)
Y=range(nmaxturn+1)
X,Y=meshgrid(X,Y)
ax.plot_wireframe(X,Y, dt_evol, rstride=2, cstride=2)

fig2 = plt.figure()
axspeed = fig2.add_subplot(111)
xspeed=range(1,n_discret+2)
yspeed=map(getspeed,xspeed)
plt.plot(xspeed,yspeed)

plt.show()
     

        



    
