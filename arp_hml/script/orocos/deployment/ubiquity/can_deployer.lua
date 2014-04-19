dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")


CanDeployer = ComposantDeployer:new();
local me = "Can1"

function CanDeployer:load()
	assert( Deployer:loadComponent(me,"arp_hml::CanOpenController"));
	assert( Deployer:addPeer("DotGraph",me))
	assert( Deployer:setActivity(me,0,40,rtt.globals.ORO_SCHED_RT));
	return true
end

function CanDeployer:connect()
	assert( Deployer:addPeer("Reporting", me));
	assert( CanDeployer:check(me));
	assert( Deployer:connect(me..".inSync", "RealTimeClock.outClock",cp))
	return true
end



