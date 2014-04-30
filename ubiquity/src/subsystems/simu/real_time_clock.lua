dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")


RealTimeClockDeployer = ComposantDeployer:new()
local me = "RealTimeClock"

function RealTimeClockDeployer:load()
	assert( Deployer:loadComponent(me,"arp_core::PeriodicClock"))
	assert( Deployer:addPeer("DotGraph",me))
	assert( Deployer:addPeer("Reporting", me))
	assert( Deployer:setActivity(me,0.010,60,rtt.globals.ORO_SCHED_RT))
	return true
end

function RealTimeClockDeployer:connect()  
	assert( RealTimeClockDeployer:check(me) )
	return true
end



