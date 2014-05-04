    # Go to Red Fire Bot
#            PreemptiveStateMachine.add('GoToRFB',
#                      AmbiOmniDirectOrder2(Pose2D(-0.400 - Robot2014.FRONT_SIDE.x, -0.600, 0), vmax=1.0),
#                      transitions={'succeeded':'PickRFB', 'timeout':'ReverseOrder'})
            
#as initial state is not the preemptive one, it is necessary to add the information here !