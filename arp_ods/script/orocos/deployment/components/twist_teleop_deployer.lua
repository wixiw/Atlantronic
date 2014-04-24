dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")

MotionControlDeployer = ComposantDeployer:new()
local me = "MotionControl"

function MotionControlDeployer:load()
	assert( Deployer:loadComponent(me,"arp_ods::TwistTeleop");
	assert( Deployer:addPeer("DotGraph",me))
	assert( Deployer:setMasterSlaveActivity("MotionScheduler", me)
	return true
end

function MotionControlDeployer:connect()
	MotionControl = Deployer:getPeer(me);
	assert( Deployer:addPeer("HmlMonitor", me)
	assert( Deployer:addPeer(me,"HmlMonitor")
	assert( Deployer:addPeer("Reporting", me)
	assert( HmlMonitor:connect(me,"inXSpeed","Joystick","outX1"))
	assert( HmlMonitor:connect(me,"inYSpeed","Joystick","outY1"))
	assert( HmlMonitor:connect(me,"inThetaSpeed","Joystick","outX2"))
	assert( HmlMonitor:connect(me,"inDeadMan","Joystick","outTrigger6"))
	assert( HmlMonitor:connect(me,"inExpertMode","Joystick","outTrigger7"))
	assert( Deployer:connect(me..".inParams","UbiquityParams.outParams",cp))
	assert( Deployer:connect(me..".inPower","HmlMonitor.outEnable",cp))
	assert( Deployer:stream(me..".inBootUpDone",ros:topic("/Strat/go"))
	
	return true
end





