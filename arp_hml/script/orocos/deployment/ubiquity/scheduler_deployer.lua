dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")


SchedulerDeployer = ComposantDeployer:new()
local me = "MotionScheduler"

function SchedulerDeployer:load()
	Deployer:loadComponent(me,"arp_hml::MotionScheduler")
	Deployer:setMasterSlaveActivity("Can1", me)
end

function SchedulerDeployer:connect()
	Deployer:connect("Can1.outClock", me..".inClock",cp)
	Scheduler = Deployer:getPeer(me)
	Scheduler:configure()
end


