dofile("/opt/ard/ubiquity/src/subsystems/orocos/component_deployer_object.lua")

MotionControlDeployer = ComposantDeployer:new()
local me = "MotionControl"

function MotionControlDeployer:load()
	assert( Deployer:loadComponent(me,"arp_ods::FreeWheel"))
	assert( Deployer:addPeer("DotGraph",me))
	assert( Deployer:setMasterSlaveActivity("MotionScheduler", me))
	return true
end


function MotionControlDeployer:connect()
	MotionControl = Deployer:getPeer(me);
	assert( Deployer:addPeer("HmlMonitor", me))
	assert( Deployer:addPeer(me,"HmlMonitor"))
	assert( Deployer:addPeer("Reporting", me))
	assert( Deployer:connect(me..".inParams","UbiquityParams.outParams",cp))
	assert( Deployer:connect(me..".inPower","HmlMonitor.outEnable",cp))
	assert( Deployer:stream(me..".inBootUpDone",ros:topic("/Strat/go"))
	return true
end





