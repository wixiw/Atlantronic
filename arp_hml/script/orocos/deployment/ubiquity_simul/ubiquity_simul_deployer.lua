dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")


UbiquitySimulDeployer = ComposantDeployer:new()

function UbiquitySimulDeployer:load()
	assert( Deployer:loadComponent("UbiquitySimul","arp_hml::UbiquitySimul"))
	assert( Deployer:setActivity("UbiquitySimul",0.010,40,rtt.globals.ORO_SCHED_OTHER))
	return true
end

function UbiquitySimulDeployer:connect()
	--je m'ajoute aux autres
	assert( Deployer:addPeer("Reporting", "UbiquitySimul"))
	assert( Deployer:addPeer("RosHmlItf", "UbiquitySimul"))

	assert( Deployer:connect("LeftDriving.outFilteredSpeedCommand","UbiquitySimul.inLeftDrivingSpeedCmd",cp) )
	assert( Deployer:connect("RightDriving.outFilteredSpeedCommand","UbiquitySimul.inRightDrivingSpeedCmd",cp) )
	assert( Deployer:connect("RearDriving.outFilteredSpeedCommand","UbiquitySimul.inRearDrivingSpeedCmd",cp) )
	assert( Deployer:connect("LeftDriving.outFilteredPositionCommand","UbiquitySimul.inLeftSteeringPositionCmd",cp) )
	assert( Deployer:connect("RightDriving.outFilteredPositionCommand","UbiquitySimul.inRightSteeringPositionCmd",cp) )
	assert( Deployer:connect("RearDriving.outFilteredPositionCommand","UbiquitySimul.inRearSteeringPositionCmd",cp) )

	assert( UbiquitySimulDeployer:check("UbiquitySimul") )

	return true
end



