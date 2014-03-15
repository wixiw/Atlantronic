dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")


SchedulerDeployer = ComposantDeployer:new()
local me = "MotionScheduler"

function SchedulerDeployer:load()
	assert( Deployer:loadComponent(me,"arp_core::MotionScheduler"))
	assert( Deployer:addPeer("DotGraph",me))
	assert( Deployer:setMasterSlaveActivity("Can1", me))
	return true
end

function SchedulerDeployer:connect()
	assert( Deployer:connect("Can1.outClock", me..".inClock",cp))
	Scheduler = assert( Deployer:getPeer(me))
	assert( Scheduler:configure())
	
	--Pas besoin d'ajouter les slaves en peer avec addPeer parce qu'ils le sont implicitment avec la fonciton setMasterSlaveActivity
	
	return true
end


