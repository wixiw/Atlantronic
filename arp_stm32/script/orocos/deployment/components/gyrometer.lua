dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")


GyrometerDeployer = ComposantDeployer:new()
local me = "Gyrometer"

function GyrometerDeployer:load()
	assert( Deployer:loadComponent(me,"arp_stm32::Gyrometer"))
	assert( Deployer:addPeer("DotGraph",me))
	assert( Deployer:setMasterSlaveActivity("Discovery", me))
	return true
end

function GyrometerDeployer:connect()
	assert( Deployer:addPeer("Reporting", me) )
	return true
end

function GyrometerDeployer:start()
	Discovery = assert(Deployer:getPeer(me))
	assert(Discovery:configure())
	assert(Discovery:start())
	return true
end


