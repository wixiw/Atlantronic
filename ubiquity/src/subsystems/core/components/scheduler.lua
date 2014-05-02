dofile("/opt/ard/ubiquity/src/subsystems/orocos/component_deployer_object.lua")


SchedulerDeployer = ComposantDeployer:new()
local me = "MotionScheduler"

function SchedulerDeployer:load()
	assert( Deployer:loadComponent(me,"arp_core::PeriodicScheduler"))
	--MasterActivity has to be set before Slaves one (else => segfault) so master activity are done here
	assert( Deployer:setActivity(me,0.0,60,rtt.globals.ORO_SCHED_RT))
	return true
end

function SchedulerDeployer:connect()
	assert( Deployer:addPeer("DotGraph",me))
	assert( Deployer:addPeer("Reporting", me))
	
	--the main scheduler is a slave connected by event port to the main timer service
	assert( Deployer:connect(me..".trigger", "MainTimer.timer_0",cp ))
	Scheduler = assert( Deployer:getPeer(me))
	return true
end

function SchedulerDeployer:start()
	print("... .... start scheduler")
	--configure first to get properties from xml 
	assert( Scheduler:configure())
	
	--then override some properties with deployment
	MotionScheduler=assert( Deployer:getPeer("MotionScheduler"))
	sched_order= assert( MotionScheduler:getProperty("sched_order"))
	sched_order:get():resize(7)
	sched_order[0]="RealTimeClock"
	sched_order[1]="UbiquitySimul"
	sched_order[2]="Odometry"
	sched_order[3]="Localizator"
	sched_order[4]="MotionControl"
	sched_order[5]="KinematicBase"
	sched_order[6]="Reporting"
	
	time_reporting = assert( MotionScheduler:getProperty("propTimeReporting"))
	time_reporting:set(false)
	
	assert(MotionScheduler:start())
	return true;
end
