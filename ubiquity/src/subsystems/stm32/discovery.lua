dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")


DiscoveryDeployer = ComposantDeployer:new()
local me = "Discovery"

function DiscoveryDeployer:load()
	assert( Deployer:loadComponent(me,"arp_stm32::Discovery"))
	assert( Deployer:addPeer("DotGraph",me))
	--TODO probleme de temps de cycle reduction de periode
	--assert( Deployer:setActivity(me,0.100,20,rtt.globals.ORO_SCHED_RT));
	assert( Deployer:setActivity(me,0.100,20,rtt.globals.ORO_SCHED_RT));
	return true
end

function DiscoveryDeployer:connect()
	assert( Deployer:addPeer("Reporting", me) )
	return true
end

function DiscoveryDeployer:start()
	Discovery = assert(Deployer:getPeer(me))
	assert(Discovery:configure())
	assert(Discovery:start())
	return true
end


