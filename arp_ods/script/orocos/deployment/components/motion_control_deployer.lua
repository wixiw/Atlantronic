dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")

MotionControlDeployer = ComposantDeployer:new()
local me = "MotionControl"

function MotionControlDeployer:load()
	Deployer:loadComponent(me,"arp_ods::MotionControl");
	assert( Deployer:addPeer("DotGraph",me))
	Deployer:setMasterSlaveActivity("MotionScheduler", me)
end


function MotionControlDeployer:connect()
	Deployer:addPeer("Reporting", me)
	RluMonitor = Deployer:getPeer("RluMonitor");
	Deployer:addPeer("RluMonitor", me);
	RluMonitor:connect(me,"inPosition","Localizator","outPose");
	RluMonitor:connect(me,"inCurrentICRSpeed","Odometry","outICRSpeed");
    RluMonitor:connect(me,"outSmoothLocNeeded","Localizator","inSmoothMode");
    
    Deployer:addPeer("HmlMonitor", me)
    Deployer:connect(me..".inParams", "UbiquityParams.outParams",cp)
    
    HmlMonitor = Deployer:getPeer("HmlMonitor");
    HmlMonitor:connect(me,"inCanPeriod","Can1","outPeriod");
end



