dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")
dofile("/opt/ard/arp_hml/script/orocos/deployment/ubiquity/motor_deployer.lua");

MotorSimulDeployer = MotorDeployer:new()

function MotorSimulDeployer:load()
	Deployer:loadComponent("LeftDriving","arp_hml::MotorSimul")
	Deployer:setActivity("LeftDriving",0,40,1)
	Deployer:loadComponent("RightDriving","arp_hml::MotorSimul")
	Deployer:setActivity("RightDriving",0,40,1)
	Deployer:loadComponent("RearDriving","arp_hml::MotorSimul")
	Deployer:setActivity("RearDriving",0,40,1)

	Deployer:loadComponent("LeftSteering","arp_hml::MotorSimul")
	Deployer:setActivity("LeftSteering",0,40,1)
	Deployer:loadComponent("RightSteering","arp_hml::MotorSimul")
	Deployer:setActivity("RightSteering",0,40,1)
	Deployer:loadComponent("RearSteering","arp_hml::MotorSimul")
	Deployer:setActivity("RearSteering",0,40,1)
end

function MotorSimulDeployer:connect()
	Deployer:addPeer("Reporting", "LeftDriving")
	Deployer:addPeer("Reporting", "RightDriving")
	Deployer:addPeer("Reporting", "RearDriving")

	Deployer:addPeer("Reporting", "LeftSteering")
	Deployer:addPeer("Reporting", "RightSteering")
	Deployer:addPeer("Reporting", "RearSteering")

	--MotorDeployer:registerToSql("LeftDriving")
	--MotorDeployer:registerToSql("RightDriving")
	--MotorDeployer:registerToSql("RearDriving")
	--MotorDeployer:registerToSql("LeftSteering")
	--MotorDeployer:registerToSql("RightSteering")
	--MotorDeployer:registerToSql("RearSteering")
	
	MotorDeployer:check("LeftDriving")
	MotorDeployer:check("RightDriving")
	MotorDeployer:check("RearDriving")
	MotorDeployer:check("LeftSteering")
	MotorDeployer:check("RightSteering")
	MotorDeployer:check("RearSteering")
end



