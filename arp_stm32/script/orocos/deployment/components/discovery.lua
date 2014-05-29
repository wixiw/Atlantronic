dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")


DiscoveryDeployer = ComposantDeployer:new()
local me = "Discovery"

function DiscoveryDeployer:load()
	assert( Deployer:loadComponent(me,"arp_stm32::Discovery"))
	assert( Deployer:addPeer("DotGraph",me))
	assert( Deployer:setActivity(me,0.100,20,rtt.globals.ORO_SCHED_RT));
	return true
end

function DiscoveryDeployer:connect()
	assert( Deployer:addPeer("Reporting", me) )
	assert( Deployer:stream(me..".outPowerStatus",      ros:topic("/Ubiquity/powerState")))
	assert( Deployer:stream(me..".inHeartbeat",         ros:topic("/Ubiquity/heartbeat")))
	assert( Deployer:stream(me..".inPowerRequest",      ros:topic("/Ubiquity/stm32_power_request")))
	
	return true
end

function DiscoveryDeployer:start()
	Discovery = assert(Deployer:getPeer(me))
	assert(Discovery:configure())
	assert(Discovery:start())
	return true
end


