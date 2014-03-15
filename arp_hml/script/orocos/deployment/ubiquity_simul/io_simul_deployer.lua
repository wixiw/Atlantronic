dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")


IoSimulDeployer = ComposantDeployer:new()

function IoSimulDeployer:load()
	assert( Deployer:loadComponent("WoodheadIn","arp_hml::WoodheadSimul"))
	assert( Deployer:addPeer("DotGraph","WoodheadIn"))
	assert( Deployer:setMasterSlaveActivity("Can1", "WoodheadIn"))

	assert( Deployer:loadComponent("WoodheadOut","arp_hml::WoodheadSimul"))
	assert( Deployer:addPeer("DotGraph","WoodheadOut"))
	assert( Deployer:setMasterSlaveActivity("Can1", "WoodheadOut"))
	
	return true
end

function IoSimulDeployer:connect()
	assert( Deployer:addPeer("WoodheadIn", "Can1"))
	assert( Deployer:addPeer("Reporting", "WoodheadIn"))

	assert( Deployer:addPeer("WoodheadOut", "Can1"))
	assert( Deployer:addPeer("Reporting", "WoodheadOut"))
	
	return true
end



