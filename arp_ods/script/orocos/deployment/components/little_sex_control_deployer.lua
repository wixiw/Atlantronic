dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")

LittleSexControlDeployer = ComposantDeployer:new()
local me = "MotionControl"

function LittleSexControlDeployer:load()
	Deployer:loadComponent(me,"arp_ods::LittleSexControl");
	Deployer:setMasterSlaveActivity("MotionScheduler", me)
end


function LittleSexControlDeployer:connect()
	Deployer:addPeer("Reporting", me)
	RluMonitor = Deployer:getPeer("RluMonitor");
	Deployer:addPeer("RluMonitor", me);
	RluMonitor:connect(me,"inPosition","Localizator","outPose");
	RluMonitor:connect(me,"inCurrentICRSpeed","Localizator","outICRSpeed");
    RluMonitor:connect(me,"outSmoothLocNeeded","Localizator","inSmoothMode");
    
    HmlMonitor = Deployer:getPeer("HmlMonitor");
    Deployer:addPeer("HmlMonitor", me)
    Deployer:connect(me..".inParams", "UbiquityParams.outParams",cp)
end



