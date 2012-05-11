dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")


LastComponentDeployer = ComposantDeployer:new()

function LastComponentDeployer:load()
	assert( Deployer:loadComponent("LastComponent","arp_master::LastComponent"))
	assert( Deployer:setActivity("LastComponent",1,0,0))
	return true

end

function LastComponentDeployer:connect()
	assert( Deployer:stream("LastComponent.outDeployed",ros:topic("/Master/deployed")))
	return true
end

function LastComponentDeployer:start()
	LastComponent = assert( Deployer:getPeer("LastComponent"))
	assert( LastComponent:configure())
	assert( LastComponent:start())
	return true
end