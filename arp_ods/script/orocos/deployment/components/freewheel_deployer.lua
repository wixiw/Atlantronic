dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")

MotionControlDeployer = ComposantDeployer:new()
local me = "MotionControl"

function MotionControlDeployer:load()
	Deployer:loadComponent(me,"arp_ods::FreeWheel");
	Deployer:setMasterSlaveActivity("MotionScheduler", me)
end


function MotionControlDeployer:connect()
	MotionControl = Deployer:getPeer(me);
	Deployer:addPeer("HmlMonitor", me)
	Deployer:addPeer(me,"HmlMonitor")
	Deployer:addPeer("Reporting", me)
	Deployer:connect(me..".inParams","UbiquityParams.outParams",cp);
	Deployer:connect(me..".inPower","HmlMonitor.outEnable",cp);
	Deployer:stream(me..".inBootUpDone",ros:topic("/Strat/go"))
	
	--Deployer:addPeer("RluMonitor", me)
	--RluMonitor = Deployer:getPeer("RluMonitor");
	--RluMonitor:connect(me,"inICRSpeed","Localizator","outICRSpeed");
end





