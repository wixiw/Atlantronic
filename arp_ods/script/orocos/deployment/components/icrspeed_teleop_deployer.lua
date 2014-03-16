dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")

MotionControlDeployer = ComposantDeployer:new()
local me = "MotionControl"

function MotionControlDeployer:load()
	Deployer:loadComponent(me,"arp_ods::IcrSpeedTeleop");
	assert( Deployer:addPeer("DotGraph",me))
	Deployer:setMasterSlaveActivity("MotionScheduler", me)
end

function MotionControlDeployer:connect()
	MotionControl = Deployer:getPeer(me);
	Deployer:addPeer("HmlMonitor", me)
	Deployer:addPeer(me,"HmlMonitor")
	Deployer:addPeer("Reporting", me)
	HmlMonitor:connect(me,"inRoSpeedCmd","Joystick","outXY1Distance");
	HmlMonitor:connect(me,"inPhiCmd","Joystick","outXY1Angle");
	HmlMonitor:connect(me,"inDeltaCmd","Joystick","outX2");
	HmlMonitor:connect(me,"inDeadMan","Joystick","outTrigger6");
	HmlMonitor:connect(me,"inExpertMode","Joystick","outTrigger7");
	HmlMonitor:connect(me,"inRotationMode","Joystick","outTrigger5");
	HmlMonitor:connect(me,"inThetaSpeed","Joystick","outY1");
	Deployer:connect(me..".inParams","UbiquityParams.outParams",cp);
	Deployer:connect(me..".inPower","HmlMonitor.outEnable",cp);
	Deployer:stream(me..".inBootUpDone",ros:topic("/Strat/go"))
	
	--Deployer:addPeer("RluMonitor", me)
	--RluMonitor = Deployer:getPeer("RluMonitor");
	--RluMonitor:connect(me,"inICRSpeed","Localizator","outICRSpeed");
end





