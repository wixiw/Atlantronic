dofile("/opt/ard/ubiquity/src/subsystems/orocos/component_deployer_object.lua")


IoDeployer = ComposantDeployer:new()

function IoDeployer:load()
	assert( Deployer:loadComponent("WoodheadIn","arp_hml::WoodheadIn"))
	assert( Deployer:addPeer("DotGraph","WoodheadIn"))
	assert( Deployer:setMasterSlaveActivity("Can1", "WoodheadIn"))

	assert( Deployer:loadComponent("WoodheadOut","arp_hml::WoodheadOut"))
	assert( Deployer:addPeer("DotGraph","WoodheadOut"))
	assert( Deployer:setMasterSlaveActivity("Can1", "WoodheadOut"))
	
	return true
end

function IoDeployer:connect()
	assert( Deployer:addPeer("WoodheadIn", "Can1"))
	assert( Deployer:addPeer("Reporting", "WoodheadIn"))

	assert( Deployer:addPeer("WoodheadOut", "Can1"))
	assert( Deployer:addPeer("Reporting", "WoodheadOut"))
	
	assert( IoDeployer:check("WoodheadIn"))
	assert( IoDeployer:check("WoodheadOut"))
	
	return true
end



