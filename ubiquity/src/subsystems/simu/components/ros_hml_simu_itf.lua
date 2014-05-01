dofile("/opt/ard/ubiquity/src/subsystems/orocos/component_deployer_object.lua")


RosHmlSimuItfDeployer = ComposantDeployer:new()

function RosHmlSimuItfDeployer:load()
	assert( Deployer:loadComponent("RosHmlItf","arp_simu::RosHmlSimuItf"))
	assert( Deployer:addPeer("DotGraph","RosHmlItf"))
	assert( Deployer:setActivity("RosHmlItf",0.050,0,rtt.globals.ORO_SCHED_OTHER))
	return true
end


function RosHmlSimuItfDeployer:connect()
	
	--assert( Deployer:addPeer("RosHmlItf", "HmlMonitor"))

--connection des ports
	
	assert( Deployer:connect("RosHmlItf.inIsHomingDone", "UbiquitySimul.outHomingDone",cp))
	assert( Deployer:connect("RosHmlItf.inRealPosition", "UbiquitySimul.outRealPosition",cp))

--connexion ROS
	
	assert( Deployer:stream("RosHmlItf.outDrivingMotorsEnable",ros:topic("/Ubiquity/driving_power")))
	assert( Deployer:stream("RosHmlItf.outSteeringMotorsEnable",ros:topic("/Ubiquity/steering_power")))
	assert( Deployer:stream("RosHmlItf.outMotorsEnable",ros:topic("/Ubiquity/motor_power")))

	assert( Deployer:stream("RosHmlItf.outWheelBlocked",ros:topic("/Ubiquity/wheel_blocked")))
	
	assert( Deployer:stream("RosHmlItf.outEmergencyStop",ros:topic("/Ubiquity/emergency_stop")))
	assert( Deployer:stream("RosHmlItf.outIsHomingDone",ros:topic("/Ubiquity/homing_done")))
	assert( Deployer:stream("RosHmlItf.outJoystickButton",ros:topic("/Ubiquity/manual_button")))
	
	assert( Deployer:stream("RosHmlItf.inBlockRobot",ros:topic("Simulation/block_robot")))
	assert( Deployer:stream("RosHmlItf.outRealLocalizationState",ros:topic("Simulation/localization_state")))
	
	return true
end
