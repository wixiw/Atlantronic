dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")


RosHmlItfDeployer = ComposantDeployer:new()

function RosHmlItfDeployer:load()
	Deployer:loadComponent("RosHmlItf","arp_hml::RosHmlItf")
	Deployer:setActivity("RosHmlItf",0.050,10,1)
end


function RosHmlItfDeployer:connectOneMotor(name)
	Deployer:addPeer("RosHmlItf", name)
	Deployer:connect("RosHmlItf.in"..name.."Blocked", name..".outMaxTorqueTimeout",cp)
end

function RosHmlItfDeployer:connect()
	
	Deployer:addPeer("RosHmlItf", "HmlMonitor")

--connection des ports
	RosHmlItfDeployer:connectOneMotor("LeftDriving")
	RosHmlItfDeployer:connectOneMotor("RightDriving")
	RosHmlItfDeployer:connectOneMotor("RearDriving")
	RosHmlItfDeployer:connectOneMotor("LeftSteering")
	RosHmlItfDeployer:connectOneMotor("RightSteering")
	RosHmlItfDeployer:connectOneMotor("RearSteering")
	

	Deployer:connect("RosHmlItf.inIoStart", "WoodheadIn.outBit1",cp)
	Deployer:connect("RosHmlItf.inIoStartColor", "WoodheadIn.outBit3",cp)
	Deployer:connect("RosHmlItf.inIoFrontLeftObstacle", "WoodheadIn.outBit4",cp)
	Deployer:connect("RosHmlItf.inIoFrontRightObstacle", "WoodheadIn.outBit7",cp)
	Deployer:connect("RosHmlItf.inIoRearObstacle", "WoodheadIn.outBit8",cp)
	
	Deployer:connect("RosHmlItf.inIsHomingDone", "HmlMonitor.outHomingDone",cp)

--connexion ROS
	
	Deployer:stream("RosHmlItf.outDrivingMotorsEnable",ros:topic("/Ubiquity/driving_power"))
	Deployer:stream("RosHmlItf.outSteeringMotorsEnable",ros:topic("/Ubiquity/steering_power"))
	Deployer:stream("RosHmlItf.outMotorsEnable",ros:topic("/Ubiquity/motor_power"))

	Deployer:stream("RosHmlItf.outWheelBlocked",ros:topic("/Ubiquity/wheel_blocked"))
	Deployer:stream("RosHmlItf.outEmergencyStop",ros:topic("/Ubiquity/emergency_stop"))
	Deployer:stream("RosHmlItf.outIsHomingDone",ros:topic("/Ubiquity/homing_done"))
	Deployer:stream("RosHmlItf.outIoStart",ros:topic("/Ubiquity/start"))
	Deployer:stream("RosHmlItf.inIoStartColor",ros:topic("/Ubiquity/color"))
	--Deployer:stream("RosHmlItf.inIoFrontLeftObstacle",ros:topic("/Ubiquity/front_left_obstacle"))
	--Deployer:stream("RosHmlItf.inIoFrontRightObstacle",ros:topic("/Ubiquity/front_right_obstacle"))
	--Deployer:stream("RosHmlItf.inIoRearObstacle",ros:topic("/Ubiquity/rear_obstacle"))
end
