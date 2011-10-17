dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")


MotorDeployer = ComposantDeployer:new()

function MotorDeployer:load()
	Deployer:loadComponent("LeftDriving","arp_hml::Faulhaber3268Bx4")
	Deployer:setActivity("LeftDriving",0,40,1)
	Deployer:loadComponent("RightDriving","arp_hml::Faulhaber3268Bx4")
	Deployer:setActivity("RightDriving",0,40,1)
	Deployer:loadComponent("RearDriving","arp_hml::Faulhaber3268Bx4")
	Deployer:setActivity("RearDriving",0,40,1)

	Deployer:loadComponent("LeftSteering","arp_hml::Faulhaber3268Bx4")
	Deployer:setActivity("LeftSteering",0,40,1)
	Deployer:loadComponent("RightSteering","arp_hml::Faulhaber3268Bx4")
	Deployer:setActivity("RightSteering",0,40,1)
	Deployer:loadComponent("RearSteering","arp_hml::Faulhaber3268Bx4")
	Deployer:setActivity("RearSteering",0,40,1)
end

function MotorDeployer:connect()
	Deployer:addPeer("LeftDriving", "Can1");
	Deployer:addPeer("Reporting", "LeftDriving")
	Deployer:addPeer("RightDriving", "Can1");
	Deployer:addPeer("Reporting", "RightDriving")
	Deployer:addPeer("RearDriving", "Can1");
	Deployer:addPeer("Reporting", "RearDriving")

	Deployer:addPeer("LeftSteering", "Can1");
	Deployer:addPeer("Reporting", "LeftSteering")
	Deployer:addPeer("RightSteering", "Can1");
	Deployer:addPeer("Reporting", "RightSteering")
	Deployer:addPeer("RearSteering", "Can1");
	Deployer:addPeer("Reporting", "RearSteering")

	Deployer:connect("LeftDriving.inSpeedCmd", "Hml.outLeftDrivingSpeedCmd",cp);
	Deployer:connect("RightDriving.inSpeedCmd", "Hml.outRightDrivingSpeedCmd",cp);
	Deployer:connect("RearDriving.inSpeedCmd", "Hml.outRearDrivingSpeedCmd",cp);
	Deployer:connect("LeftSteering.inSpeedCmd", "Hml.outLeftSteeringSpeedCmd",cp);
	Deployer:connect("RightSteering.inSpeedCmd", "Hml.outRightSteeringSpeedCmd",cp);
	Deployer:connect("RearSteering.inSpeedCmd", "Hml.outRearSteeringSpeedCmd",cp);
end

function MotorDeployer:start()
	LeftDriving = Deployer:getPeer("LeftDriving")
	RightDriving = Deployer:getPeer("RightDriving")
	RearDriving = Deployer:getPeer("RearDriving")
	LeftSteering = Deployer:getPeer("LeftSteering")
	RightSteering = Deployer:getPeer("RightSteering")
	RearSteering = Deployer:getPeer("RearSteering")

	LeftDriving:configure()
	RightDriving:configure()
	RearDriving:configure()
	LeftSteering:configure()
	RightSteering:configure()
	RearSteering:configure()

	LeftDriving:start()
	RightDriving:start()
	RearDriving:configure()
	LeftSteering:start()
	RightSteering:start()
	RearSteering:configure()
end



