dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")


RosHmlItfDeployer = ComposantDeployer:new()

function RosHmlItfDeployer:load()
	assert( Deployer:loadComponent("RosHmlItf","arp_hml::RosHmlItf"))
	assert( Deployer:addPeer("DotGraph","RosHmlItf"))
	assert( Deployer:setActivity("RosHmlItf",0.050,0,rtt.globals.ORO_SCHED_OTHER))
	return true
end


function RosHmlItfDeployer:connectOneMotor(name)
	assert( Deployer:addPeer("RosHmlItf", name))
	assert( Deployer:connect("RosHmlItf.in"..name.."Blocked", name..".outMaxTorqueTimeout",cp))
	return true
end

function RosHmlItfDeployer:connect()
	
	assert( Deployer:addPeer("RosHmlItf", "HmlMonitor"))

--connection des ports
	assert( RosHmlItfDeployer:connectOneMotor("LeftDriving"))
	assert( RosHmlItfDeployer:connectOneMotor("RightDriving"))
	assert( RosHmlItfDeployer:connectOneMotor("RearDriving"))
	assert( RosHmlItfDeployer:connectOneMotor("LeftSteering"))
	assert( RosHmlItfDeployer:connectOneMotor("RightSteering"))
	assert( RosHmlItfDeployer:connectOneMotor("RearSteering"))
	
	assert( Deployer:connect("RosHmlItf.inIsHomingDone", "HmlMonitor.outHomingDone",cp))
	
	assert( Deployer:connect("RosHmlItf.inButton1", "Joystick.outButton1",cp))
	assert( Deployer:connect("RosHmlItf.inButton2", "Joystick.outButton2",cp))
	assert( Deployer:connect("RosHmlItf.inButton3", "Joystick.outButton3",cp))
	assert( Deployer:connect("RosHmlItf.inButton4", "Joystick.outButton4",cp))
	assert( Deployer:connect("RosHmlItf.inButton9", "Joystick.outButton9",cp))
	assert( Deployer:connect("RosHmlItf.inButton10", "Joystick.outButton10",cp))

--connexion ROS
	
	assert( Deployer:stream("RosHmlItf.outDrivingMotorsEnable",ros:topic("/Ubiquity/driving_power")))
	assert( Deployer:stream("RosHmlItf.outSteeringMotorsEnable",ros:topic("/Ubiquity/steering_power")))
	assert( Deployer:stream("RosHmlItf.outMotorsEnable",ros:topic("/Ubiquity/motor_power")))

	assert( Deployer:stream("RosHmlItf.outWheelBlocked",ros:topic("/Ubiquity/wheel_blocked")))
	assert( Deployer:stream("RosHmlItf.inBlockRobot",ros:topic("Simul/block_robot")))
	
	assert( Deployer:stream("RosHmlItf.outEmergencyStop",ros:topic("/Ubiquity/emergency_stop")))
	assert( Deployer:stream("RosHmlItf.outIsHomingDone",ros:topic("/Ubiquity/homing_done")))
	assert( Deployer:stream("RosHmlItf.outIoStart",ros:topic("/Ubiquity/start")))
	assert( Deployer:stream("RosHmlItf.outIoStartColor",ros:topic("/Ubiquity/color")))
	assert( Deployer:stream("RosHmlItf.outFrontLeftObstacle",ros:topic("/Ubiquity/front_left_obstacle")))
	assert( Deployer:stream("RosHmlItf.outFrontRightObstacle",ros:topic("/Ubiquity/front_right_obstacle")))
	assert( Deployer:stream("RosHmlItf.outRearObstacle",ros:topic("/Ubiquity/rear_obstacle")))
	assert( Deployer:stream("RosHmlItf.outJoystickButton",ros:topic("/Ubiquity/manual_button")))
	
	return true
end
