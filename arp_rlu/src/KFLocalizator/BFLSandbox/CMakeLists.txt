###################################################################################################################
#    EDIT BELOW THIS LINE AND UNTIL NEXT LINE FOR A NEW PROJECT

            
rosbuild_add_executable(TestLinearKalman LinearKalman/test_linear_kalman.cpp mobile_robot.cpp nonlinearanalyticconditionalgaussianmobile.cpp)
rosbuild_add_executable(TestNonlinearKalman NonlinearKalman/test_nonlinear_kalman.cpp mobile_robot.cpp nonlinearanalyticconditionalgaussianmobile.cpp)
rosbuild_add_executable(TestNonlinearParticle NonlinearParticle/test_nonlinear_particle.cpp mobile_robot.cpp nonlinearanalyticconditionalgaussianmobile.cpp NonlinearParticle/nonlinearMeasurementPdf.cpp NonlinearParticle/nonlinearSystemPdf.cpp)


###################################################################################################################