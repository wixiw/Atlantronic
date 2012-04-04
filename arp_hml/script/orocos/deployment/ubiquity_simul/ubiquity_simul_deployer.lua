dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")


UbiquitySimulDeployer = ComposantDeployer:new()

function UbiquitySimulDeployer:load()
	Deployer:loadComponent("UbiquitySimul","arp_hml::UbiquitySimul")
	Deployer:setActivity("UbiquitySimul",0.010,40,1)
end

function UbiquitySimulDeployer:registerToSql(name)
	OrocosSqlMonitor = Deployer:getPeer("OrocosSqlBridge")
	Deployer:addPeer("OrocosSqlBridge",name)
end

function UbiquitySimulDeployer:connect()
	--je m'ajoute aux autres
	Deployer:addPeer("Reporting", "UbiquitySimul")
	Deployer:addPeer("RosHmlItf", "UbiquitySimul")
	
	--UbiquitySimulDeployer:registerToSql("LeftDriving")
	--UbiquitySimulDeployer:registerToSql("RightDriving")
	--UbiquitySimulDeployer:registerToSql("RearDriving")
	--UbiquitySimulDeployer:registerToSql("LeftSteering")
	--UbiquitySimulDeployer:registerToSql("RightSteering")
	--UbiquitySimulDeployer:registerToSql("RearSteering")

	Deployer:connect("LeftDriving.outFilteredSpeedCommand","UbiquitySimul.inLeftDrivingSpeedCmd",cp)
	Deployer:connect("RightDriving.outFilteredSpeedCommand","UbiquitySimul.inRightDrivingSpeedCmd",cp)
	Deployer:connect("RearDriving.outFilteredSpeedCommand","UbiquitySimul.inRearDrivingSpeedCmd",cp)
	Deployer:connect("LeftDriving.outFilteredPositionCommand","UbiquitySimul.inLeftSteeringPositionCmd",cp)
	Deployer:connect("RightDriving.outFilteredPositionCommand","UbiquitySimul.inRightSteeringPositionCmd",cp)
	Deployer:connect("RearDriving.outFilteredPositionCommand","UbiquitySimul.inRearSteeringPositionCmd",cp)

	UbiquitySimulDeployer:check("UbiquitySimul")

end



