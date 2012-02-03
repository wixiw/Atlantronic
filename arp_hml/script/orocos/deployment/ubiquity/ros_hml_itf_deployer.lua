dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")


RosHmlItfDeployer = ComposantDeployer:new()

function RosHmlItfDeployer:load()
	Deployer:loadComponent("RosHmlItf","arp_hml::RosHmlItf")
	Deployer:setActivity("RosHmlItf",0.050,10,1)
end


function RosHmlItfDeployer:connectOneMotor(name)
	--Deployer:addPeer("RosHmlItf", name)
	Deployer:connect("RosHmlItf.in"..name.."Position", name..".outMeasuredPosition",cp)
	Deployer:connect("RosHmlItf.in"..name.."PositionTime", name..".outMeasuredPositionTime",cp)
	Deployer:connect("RosHmlItf.in"..name.."SpeedMeasure", name..".outComputedSpeed",cp)
	Deployer:connect("RosHmlItf.in"..name.."Blocked", name..".outMaxTorqueTimeout",cp)
end

function RosHmlItfDeployer:connect()
	
	Deployer:addPeer("RosHmlItf", "HmlMonitor")

--connection des ports
	--RosHmlItfDeployer:connectOneMotor("LeftDriving")
	--RosHmlItfDeployer:connectOneMotor("RightDriving")
	RosHmlItfDeployer:connectOneMotor("RearDriving")
	--RosHmlItfDeployer:connectOneMotor("LeftSteering")
	--RosHmlItfDeployer:connectOneMotor("RightSteering")
	RosHmlItfDeployer:connectOneMotor("RearSteering")

	Deployer:connect("RosHmlItf.inIoStart", "WoodheadIn.outBit1",cp)

--connexion ROS
	cpRos.name_id="/Ubiquity/omnidirectional_command"
	Deployer:stream("RosHmlItf.inOmniCmd",cpRos)
	cpRos.name_id="/Ubiquity/odo"
	Deployer:stream("RosHmlItf.outOdometryMeasures",cpRos)
	cpRos.name_id="/Ubiquity/omnidirectional_measure"
	Deployer:stream("RosHmlItf.outOmniSpeedMeasure",cpRos)

	cpRos.name_id="/Ubiquity/driving_power"
	Deployer:stream("RosHmlItf.outDrivingMotorsEnable",cpRos)
	cpRos.name_id="/Ubiquity/steering_power"
	Deployer:stream("RosHmlItf.outSteeringMotorsEnable",cpRos)
	cpRos.name_id="/Ubiquity/motor_power"
	Deployer:stream("RosHmlItf.outMotorsEnable",cpRos)

	cpRos.name_id="/Ubiquity/wheel_blocked"
	Deployer:stream("RosHmlItf.outWheelBlocked",cpRos)
	cpRos.name_id="/Ubiquity/emergency_stop"
	Deployer:stream("RosHmlItf.outEmergencyStop",cpRos)
	cpRos.name_id="/Ubiquity/start"
	Deployer:stream("RosHmlItf.outIoStart",cpRos)
end
