dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")


MotorDeployer = ComposantDeployer:new()

function MotorDeployer:load()
	--Deployer:loadComponent("LeftDriving","arp_hml::Faulhaber3268Bx4")
	--Deployer:setActivity("LeftDriving",0,40,1)
	--Deployer:loadComponent("RightDriving","arp_hml::Faulhaber3268Bx4")
	--Deployer:setActivity("RightDriving",0,40,1)
	Deployer:loadComponent("RearDriving","arp_hml::Faulhaber3268Bx4")
	Deployer:setActivity("RearDriving",0,40,1)

	--Deployer:loadComponent("LeftSteering","arp_hml::Faulhaber3268Bx4")
	--Deployer:setActivity("LeftSteering",0,40,1)
	--Deployer:loadComponent("RightSteering","arp_hml::Faulhaber3268Bx4")
	--Deployer:setActivity("RightSteering",0,40,1)
	Deployer:loadComponent("RearSteering","arp_hml::Faulhaber3268Bx4")
	Deployer:setActivity("RearSteering",0,40,1)
end

function MotorDeployer:connect()
	--Deployer:addPeer("LeftDriving", "Can1");
	--Deployer:addPeer("Reporting", "LeftDriving")
	--Deployer:addPeer("RightDriving", "Can1");
	--Deployer:addPeer("Reporting", "RightDriving")
	Deployer:addPeer("RearDriving", "Can1");
	Deployer:addPeer("Reporting", "RearDriving")

	--Deployer:addPeer("LeftSteering", "Can1");
	--Deployer:addPeer("Reporting", "LeftSteering")
	--Deployer:addPeer("RightSteering", "Can1");
	--Deployer:addPeer("Reporting", "RightSteering")
	Deployer:addPeer("RearSteering", "Can1");
	Deployer:addPeer("Reporting", "RearSteering")

	--Deployer:connect("LeftDriving.inSpeedCmd", "RosHmlItf.outLeftDrivingSpeedCmd",cp);
	--Deployer:connect("RightDriving.inSpeedCmd", "RosHmlItf.outRightDrivingSpeedCmd",cp);
	Deployer:connect("RearDriving.inSpeedCmd", "RosHmlItf.outRearDrivingSpeedCmd",cp);
	--Deployer:connect("LeftSteering.inPositionCmd", "RosHmlItf.outLeftSteeringPositionCmd",cp);
	--Deployer:connect("RightSteering.inPositionCmd", "RosHmlItf.outRightSteeringPositionCmd",cp);
	Deployer:connect("RearSteering.inPositionCmd", "RosHmlItf.outRearSteeringPositionCmd",cp);
end



