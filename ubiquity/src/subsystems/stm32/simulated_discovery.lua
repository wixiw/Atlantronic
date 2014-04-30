dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")


SimulatedDiscoveryDeployer = ComposantDeployer:new()
local me = "Discovery"

function SimulatedDiscoveryDeployer:load()
	assert( Deployer:loadComponent(me,"arp_stm32::SimulatedDiscovery"))
	assert( Deployer:addPeer("DotGraph",me))
	assert( Deployer:setActivity(me,0.0,20,rtt.globals.ORO_SCHED_RT));
	return true
end

function SimulatedDiscoveryDeployer:connect()
	assert( Deployer:addPeer("Reporting", me) )
	return true
end

function SimulatedDiscoveryDeployer:start()
	Discovery = assert(Deployer:getPeer(me))
	assert(Discovery:configure())
	assert(Discovery:start())
	return true
end


