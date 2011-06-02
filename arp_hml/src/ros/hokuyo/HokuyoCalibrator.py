#!/usr/bin/env python
import roslib; roslib.load_manifest('arp_hml')
import rospy
from sensor_msgs.msg import LaserScan
from arp_hml.srv import *
import numpy as np
import matplotlib.pyplot as plt

class HokuyoCalibrator():
    
    def __init__(self):
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber("/scan", LaserScan, self.callback)
        s = rospy.Service('grabbDataWhite', GrabbData, self.handle_grabbDataWhite)
        s = rospy.Service('grabbDataBlack', GrabbData, self.handle_grabbDataBlack)
        s = rospy.Service('computeModel', ComputeModel, self.handle_computeModel)
        
        self.lastMeasures = np.zeros((20 , 1))  #les 20 dernieres mesures
        self.meanMeasuresWhite = []
        self.meanMeasuresBlack = []
        self.groundTruthsWhite = []
        self.groundTruthsBlack = []
        
        rospy.spin()

    def callback(self, data):
        self.lastMeasures = np.vstack((data.ranges[ len(data.ranges) / 2 ], self.lastMeasures[0:9, :]))
    
    def handle_grabbDataWhite(self, req):
        if self.meanMeasuresWhite == []:
            self.meanMeasuresWhite = np.array([ np.mean(self.lastMeasures) ])
            self.groundTruthsWhite = np.array([ req.real ])
        else:
            self.meanMeasuresWhite = np.vstack((self.meanMeasuresWhite, np.array([ np.mean(self.lastMeasures) ])))
            self.groundTruthsWhite = np.vstack((self.groundTruthsWhite, np.array([ req.real ])))
        
        rospy.loginfo(rospy.get_name() + " grabb data white !  groundTruth=%f, meanMeasuresWhite=%f", req.real, np.mean(self.lastMeasures))
        return GrabbDataResponse(np.mean(self.lastMeasures))
    
    def handle_grabbDataBlack(self, req):
        if self.meanMeasuresBlack == []:
            self.meanMeasuresBlack = np.array([ np.mean(self.lastMeasures) ])
            self.groundTruthsBlack = np.array([ req.real ])
        else:
            self.meanMeasuresBlack = np.vstack((self.meanMeasuresBlack, np.array([ np.mean(self.lastMeasures) ])))
            self.groundTruthsBlack = np.vstack((self.groundTruthsBlack, np.array([ req.real ])))
        
        rospy.loginfo(rospy.get_name() + " grabb data black !  groundTruth=%f, meanMeasuresBlack=%f", req.real, np.mean(self.lastMeasures))
        return GrabbDataResponse(np.mean(self.lastMeasures))
    
    def handle_computeModel(self, req):
        print "GroundTruths=", self.groundTruths
        print "MeanMeasuresWhite=", self.meanMeasuresWhite
        print "MeanMeasuresBlack=", self.meanMeasuresBlack
        
#        threshold = 0.015
#
#        nbMeas = len(self.meanMeasures)
#        
#        filtMeas = []
#        filtGrTr = []
#        for i in range(nbMeas):
#            if np.abs(self.meanMeasures[i] - self.groundTruths[i]) < threshold:
#                if filtMeas == []:
#                    filtMeas = self.meanMeasures[i]
#                    filtGrTr = self.groundTruths[i]
#                else:
#                    filtMeas = np.hstack((filtMeas, self.meanMeasures[i]))
#                    filtGrTr = np.hstack((filtGrTr, self.groundTruths[i]))
#                
#        print nbMeas - len(filtMeas), "outlier(s) ejected"    
#        nbMeas = len(filtMeas)
#        
#        model = np.polyfit(filtMeas, filtGrTr, 1)
#        print "Model coeff:", model
#        rectifior = np.poly1d(model)
#        
#        residuals = rectifior(filtMeas) - filtGrTr
#        print "mean residuals:", np.mean(residuals) 
#        print "std residuals:", np.std(residuals) 
#        
#        f = plt.figure(0)
#        xp = np.linspace(0., 2., 100)
#        plt.plot(filtGrTr , filtMeas, 'ro')
#        plt.plot(xp , xp, 'b--')
#        plt.plot(filtGrTr , rectifior(filtMeas), 'gx')
#        plt.title("Calib")
#        plt.xlabel('real in m')
#        plt.ylabel('meas in m')
#        
#        f = plt.figure(1)
#        plt.plot(filtGrTr , residuals, 'ro')
#        plt.title("Residual")
#        plt.xlabel('ground truth in m')
#        plt.ylabel('residual in m')
#        
#        plt.show()
#        
#        return ComputeModelResponse(a, b)
    
        return ComputeModelResponse(0., 0.)

    

if __name__ == '__main__':
    try:
        HokuyoCalibrator()
    except rospy.ROSInterruptException: pass
    
