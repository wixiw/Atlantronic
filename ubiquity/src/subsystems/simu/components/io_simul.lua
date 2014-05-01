dofile("/opt/ard/ubiquity/src/subsystems/orocos/component_deployer_object.lua")


IoSimulDeployer = ComposantDeployer:new()

function IoSimulDeployer:load()
	assert( Deployer:loadComponent("WoodheadIn","arp_hml::WoodheadSimul"))
	assert( Deployer:addPeer("DotGraph","WoodheadIn"))
	assert( Deployer:setMasterSlaveActivity("MotionControler", "WoodheadIn"))

	assert( Deployer:loadComponent("WoodheadOut","arp_hml::WoodheadSimul"))
	assert( Deployer:addPeer("DotGraph","WoodheadOut"))
	assert( Deployer:setMasterSlaveActivity("MotionControler", "WoodheadOut"))
	
	return true
end

function IoSimulDeployer:connect()
	assert( Deployer:addPeer("WoodheadIn", "Can1"))
	assert( Deployer:addPeer("Reporting", "WoodheadIn"))

	assert( Deployer:addPeer("WoodheadOut", "Can1"))
	assert( Deployer:addPeer("Reporting", "WoodheadOut"))
	
	return true
end



