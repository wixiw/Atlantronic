# A utiliser pour aller quelque part sans se payer le decor
            
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


            PreemptiveStateMachine.add('GoDirect',
                      AmbiOmniDirectOrder(x,y,theta), 
                      transitions={'succeeded':'succeeded', 'timeout':'problem'})

            PreemptiveStateMachine.add('GoNextHalf',
                      GoInHalf(getTableHalf(x,y,Data.color)), 
                      transitions={'succeeded':'succeeded', 'timeout':'problem'})
            
          #todo  
            PreemptiveStateMachine.add('GoDiagHalf',
                      AmbiOmniDirectOrder(x,y,theta), 
                      transitions={'succeeded':'succeeded', 'timeout':'problem'})

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

class GoInHalf(CyclicState):
        def __init__(self, table_half):
            CyclicState.__init__(self, outcomes=['succeeded'])

        def executeTransitions(self):
            if  robot_table == "closeTop":
                target = Point(800,700)
            elif  robot_table == "closeBot":
                target = Point(800,-700)
            elif  robot_table == "farBot":
                target = Point(-800,-700)
            elif  robot_table == "farTop":
                target = Point(-800,700)
            self.omnidirect(target.x, target.y, target.theta)
