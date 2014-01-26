dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")


DiscoveryDeployer = ComposantDeployer:new()

function DiscoveryDeployer:load()
	assert( Deployer:loadComponent("Discovery","arp_stm32::Discovery"))
	return true
end

function DiscoveryDeployer:connect()
	
	return true
end

function DiscoveryDeployer:start()
	
	return true
end


