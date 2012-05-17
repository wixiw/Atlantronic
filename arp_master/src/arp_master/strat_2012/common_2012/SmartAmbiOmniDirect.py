# A utiliser pour aller quelque part sans se payer le decor
            
class SmartAmbiOmniDirectOrder(MotionState):
    def __init__(self,x,y,theta, vmax=-1.0):
        MotionState.__init__(self)
        self.x = x
        self.y = y
        self.theta = theta
        self.vmax = vmax
        
    def createAction(self):
        self.pose = AmbiPoseRed(self.x, self.y, self.theta, Data.color)
        self.omnidirect(self.pose.x, self.pose.y, self.pose.theta, self.vmax)