dofile("/opt/ard/ubiquity/src/subsystems/orocos/component_deployer_object.lua")


TimeDeployer = ComposantDeployer:new()
--MAIN_TIMER_PERIOD = 0.010
MAIN_TIMER_PERIOD = 0.100
DISCOVERY_TIMER_PERIOD = 0.100
local timers = "MainTimer"
local rtc = "RealTimeClock"

function TimeDeployer:load(simulation)
	if simulation == 0 
	then
		assert( Deployer:loadComponent(rtc,"arp_core::RealTimeClock"))
	else
		assert( Deployer:loadComponent(rtc,"arp_core::SimulatedRtc"))
	end
	return true
end

function TimeDeployer:connect()  
	assert( Deployer:addPeer("DotGraph",rtc))
	assert( Deployer:addPeer("Reporting", rtc))
	assert( Deployer:setMasterSlaveActivity("MotionScheduler", rtc) )
	
	assert( Deployer:loadComponent(timers,"OCL::TimerComponent"))
	assert( Deployer:addPeer("DotGraph",timers))
	assert( Deployer:addPeer("Reporting", timers))
	assert( Deployer:setActivity(timers,0.0,70,rtt.globals.ORO_SCHED_RT))

	assert(TimeDeployer:check(timers))
	assert(TimeDeployer:check(rtc))
	return true
end

function TimeDeployer:start()
	print("... .... start time")
	MainTimer = assert(Deployer:getPeer(timers))
	timerId = 0
	assert(MainTimer:startTimer(timerId, MAIN_TIMER_PERIOD))
	timerId = timerId + 1
	assert(MainTimer:startTimer(timerId, DISCOVERY_TIMER_PERIOD))
	timerId = timerId + 1
	MainTimer:setMaxTimers(timerId)
	assert(MainTimer:configure())
	assert(MainTimer:start())
	
		
	RealTimeClock = assert(Deployer:getPeer(rtc))
	assert(RealTimeClock:configure())
	assert(RealTimeClock:start())
	return true
end