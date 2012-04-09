dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")

RosOdsItfDeployer = ComposantDeployer:new()
local me = "RosOdsItf"

function RosOdsItfDeployer:load()
	Deployer:loadComponent(me,"arp_ods::RosOdsItf");
	Deployer:setActivity(me, 0.050, 10, 1)
end


function RosOdsItfDeployer:connect()
	Deployer:connect(me..".inCurrentOrderIsFinished","MotionControl.outOrderFinished",cp)
	Deployer:connect(me..".inCurrentOrderIsInError","MotionControl.outOrderInError",cp)
	Deployer:addPeer(me,"MotionControl");
	
	RluMonitor = Deployer:getPeer("RluMonitor");
	Deployer:addPeer("RluMonitor", me);
	RluMonitor:connect(me,"inPose","Localizator","outPose");
end



