dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")
dofile("/opt/ard/arp_hml/script/orocos/deployment/ubiquity/ros_hml_itf_deployer.lua");

RosHmlItfSimulDeployer = RosHmlItfDeployer:new()

function RosHmlItfDeployer:load()
	Deployer:loadComponent("RosHmlItf","arp_hml::RosHmlItf")
	Deployer:setActivity("RosHmlItf",0.050,10,1)
end


function RosHmlItfDeployer:connect()
	
	Deployer:addPeer("RosHmlItf", "HmlMonitor")

--connection des ports
	--[[
	RosHmlItfDeployer:connectOneMotor("LeftDriving")
	RosHmlItfDeployer:connectOneMotor("RightDriving")
	RosHmlItfDeployer:connectOneMotor("RearDriving")
	RosHmlItfDeployer:connectOneMotor("LeftSteering")
	RosHmlItfDeployer:connectOneMotor("RightSteering")
	RosHmlItfDeployer:connectOneMotor("RearSteering")

	Deployer:connect("RosHmlItf.inIoStart", "WoodheadIn.outBit1",cp)
	]]
	
	Deployer:connect("RosHmlItf.inRealPosition", "UbiquitySimul.outRealPosition",cp)
	
--connexion ROS
	Deployer:stream("RosHmlItf.inOmniCmd",ros:topic("/Ubiquity/omnidirectional_command"))
	Deployer:stream("RosHmlItf.outOdometryMeasures",ros:topic("/Ubiquity/odo"))
	Deployer:stream("RosHmlItf.outOmniSpeedMeasure",ros:topic("/Ubiquity/omnidirectional_measure"))

	Deployer:stream("RosHmlItf.outDrivingMotorsEnable",ros:topic("/Ubiquity/driving_power"))
	Deployer:stream("RosHmlItf.outSteeringMotorsEnable",ros:topic("/Ubiquity/steering_power"))
	Deployer:stream("RosHmlItf.outMotorsEnable",ros:topic("/Ubiquity/motor_power"))

	Deployer:stream("RosHmlItf.outWheelBlocked",ros:topic("/Ubiquity/wheel_blocked"))
	Deployer:stream("RosHmlItf.outEmergencyStop",ros:topic("/Ubiquity/emergency_stop"))
	Deployer:stream("RosHmlItf.outIoStart",ros:topic("/Ubiquity/start"))
	
	Deployer:stream("RosHmlItf.outRealPosition",ros:topic("/Ubiquity/simuRealPose"))
end
