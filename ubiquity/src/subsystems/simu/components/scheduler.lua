dofile("/opt/ard/ubiquity/src/subsystems/orocos/component_deployer_object.lua")


SchedulerDeployer = ComposantDeployer:new()
local me = "MotionScheduler"

function SchedulerDeployer:load()
	assert( Deployer:loadComponent(me,"arp_core::MotionScheduler"))
	assert( Deployer:addPeer("DotGraph",me))
	assert( Deployer:addPeer("UbiquitySimul",me))
	assert( Deployer:setMasterSlaveActivity("UbiquitySimul", me))
	return true
end

function SchedulerDeployer:connect()
	assert( Deployer:connect("RealTimeClock.outClock", me..".inClock",cp))
	Scheduler = assert( Deployer:getPeer(me))
	assert( Scheduler:configure())
	
	--Pas besoin d'ajouter les slaves en peer avec addPeer parce qu'ils le sont implicitment avec la fonciton setMasterSlaveActivity
	MotionScheduler=assert( Deployer:getPeer("MotionScheduler"))
	sched_order= assert( MotionScheduler:getProperty("sched_order"))
	sched_order:get():resize(4)
	sched_order[0]="Odometry"
	sched_order[1]="Localizator"
	sched_order[2]="MotionControl"
	sched_order[3]="KinematicBase"
	return true
end


