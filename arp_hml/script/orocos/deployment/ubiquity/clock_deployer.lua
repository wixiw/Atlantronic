dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")


ClockDeployer = ComposantDeployer:new();
local me = "RealTimeClock"

function ClockDeployer:load()
	assert( Deployer:loadComponent(me,"arp_core::PeriodicClock"));
	assert( Deployer:addPeer("DotGraph",me))
	assert( Deployer:addPeer("Reporting", me));
	assert( Deployer:setActivity(me,0.010,65,rtt.globals.ORO_SCHED_RT));
	return true
end

function ClockDeployer:connect()
	return true
end


function ClockDeployer:start()
	RealTimeClock = assert(Deployer:getPeer(me))
	assert(RealTimeClock:configure())
	assert(RealTimeClock:start())
	return true
end


