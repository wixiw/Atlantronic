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
        
        
class SmartAmbiOmniDirectOrder(PreemptiveStateMachine):
    def __init__(self,x,y,theta, vmax=-1.0):
        PreemptiveStateMachine.__init__(self,outcomes=['succeeded','problem'])
        with self:      
            PreemptiveStateMachine.addPreemptive('EndMatchPreemption',
                                             EndMatchPreempter(-Robot2012.END_GAME_DELAY),
                                             transitions={'endMatch':'end'})
            
            PreemptiveStateMachine.add('ChooseState',
                      ChooseState(), 
                      transitions={'inHalfTable':'GoDirect', 'inNearHalfTable':'GoNextHalf', 'inDiagHalfTable':'GoDiagHalf'})
            self.setInitialState('ChooseState')


            PreemptiveStateMachine.add('ChooseState',
                      ChooseState, 
                      transitions={'inHalfTable':'GoDirect', 'inNearHalfTable':'GoNextHalf', 'inDiagHalfTable':'GoDiagHalf'})
            self.setInitialState('ChooseState')




class ChooseState(CyclicState):
    def __init__(self):
        CyclicState.__init__(self, outcomes=['inHalfTable','inNearHalfTable','inDiagHalfTable'])

    def executeTransitions(self):
        robot_table_half = Table2012.getTableHalf(Inputs.getx(),Inputs.gety(),Data.color)
        target_table_half = Table2012.getTableHalf(self.x,self.y,Data.color)
        if  robot_table_half == target_table_half: 
            return 'inHalfTable'
        elif robot_table_half == Table2012.getOppositeHalf():
            return 'inDiagHalfTable'
        else:
            return 'inNearHalfTable'
