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
        s = rospy.Service('grabbData', GrabbData, self.handle_grabbData)
        s = rospy.Service('computeModel', ComputeModel, self.handle_computeModel)
        
        self.lastMeasure = np.zeros((20 , 1))
        self.meanMeasures = []
        self.groundTruths = []
        
        rospy.spin()

    def callback(self, data):
        self.lastMeasure = np.vstack((data.ranges[ len(data.ranges) / 2 ], self.lastMeasure[0:9, :]))
    
    def handle_grabbData(self, req):
        rospy.loginfo(rospy.get_name() + " grabb data !  groundTruth=%f, measure=%f", req.real, np.mean(self.lastMeasure))
        if self.meanMeasures == []:
            self.meanMeasures = np.array([ np.mean(self.lastMeasure) ])
            self.groundTruths = np.array([ req.real ])
        else:
            self.meanMeasures = np.vstack((self.meanMeasures, np.array([ np.mean(self.lastMeasure) ])))
            self.groundTruths = np.vstack((self.groundTruths, np.array([ req.real ])))
        return GrabbDataResponse(np.mean(self.lastMeasure))
    
    def handle_computeModel(self, req):
        print "GroundTruths=", self.groundTruths
        print "MeanMeasures=", self.meanMeasures
        
        threshold = 0.015

        nbMeas = len(self.meanMeasures)
        
        filtMeas = []
        filtGrTr = []
        for i in range(nbMeas):
            if np.abs(self.meanMeasures[i] - self.groundTruths[i]) < threshold:
                if filtMeas == []:
                    filtMeas = self.meanMeasures[i]
                    filtGrTr = self.groundTruths[i]
                else:
                    filtMeas = np.hstack((filtMeas, self.meanMeasures[i]))
                    filtGrTr = np.hstack((filtGrTr, self.groundTruths[i]))
                
        print nbMeas - len(filtMeas), "outlier(s) ejected"    
        nbMeas = len(filtMeas)
        
        model = np.polyfit(filtMeas, filtGrTr, 1)
        print "Model coeff:", model
        rectifior = np.poly1d(model)
        
        residuals = rectifior(filtMeas) - filtGrTr
        print "mean residuals:", np.mean(residuals) 
        print "std residuals:", np.std(residuals) 
        
        f = plt.figure(0)
        xp = np.linspace(0., 2., 100)
        plt.plot(filtGrTr , filtMeas, 'ro')
        plt.plot(xp , xp, 'b--')
        plt.plot(filtGrTr , rectifior(filtMeas), 'gx')
        plt.title("Calib")
        plt.xlabel('real in m')
        plt.ylabel('meas in m')
        
        f = plt.figure(1)
        plt.plot(filtGrTr , residuals, 'ro')
        plt.title("Residual")
        plt.xlabel('ground truth in m')
        plt.ylabel('residual in m')
        
        plt.show()
        
        return ComputeModelResponse(a, b)

    

if __name__ == '__main__':
    try:
        HokuyoCalibrator()
    except rospy.ROSInterruptException: pass
    
