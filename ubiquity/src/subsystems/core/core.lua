dofile("/opt/ard/ubiquity/src/subsystems/orocos/component_deployer_object.lua")


CoreDeployer = ComposantDeployer:new()
local rtc = "RealTimeClock"
local paramServer = "UbiquityParams"

function CoreDeployer:load()
	print("... LOAD core")
	assert( Deployer:import("arp_core"))

	assert( Deployer:loadComponent(rtc,"arp_core::PeriodicClock"))
	assert( Deployer:addPeer("DotGraph",rtc))
	assert( Deployer:addPeer("Reporting", rtc))
	assert( Deployer:setActivity(rtc,0.010,65,rtt.globals.ORO_SCHED_RT))
	
	assert( Deployer:loadComponent(paramServer,"arp_core::ParamsComponent"))
	assert( Deployer:addPeer("DotGraph",paramServer))
	assert( Deployer:setActivity(paramServer,0.100,0,rtt.globals.ORO_SCHED_OTHER))
	return true
end

function CoreDeployer:connect()
	print("... CONNECT core")
	return true
end


function CoreDeployer:start()
	print("... START core")
	RealTimeClock = assert(Deployer:getPeer(rtc))
	assert(RealTimeClock:configure())
	assert(RealTimeClock:start())
	
	UbiquityParams = assert(Deployer:getPeer(paramServer))
	assert( UbiquityParams:configure())
	assert( UbiquityParams:start())
	
	return true
end
