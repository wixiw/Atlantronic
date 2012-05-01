dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")

LittleSexControlDeployer = ComposantDeployer:new()
local me = "MotionControl"

function LittleSexControlDeployer:load()
	Deployer:loadComponent(me,"arp_ods::LittleSexControl");
	Deployer:setActivity(me,0.0,25,1);
end


function LittleSexControlDeployer:registerToSql()
	OrocosSqlMonitor = Deployer:getPeer("OrocosSqlBridge")
	Deployer:addPeer("OrocosSqlBridge",me)
end

function LittleSexControlDeployer:connect()
	Deployer:addPeer("Reporting", me)
	RluMonitor = Deployer:getPeer("RluMonitor");
	Deployer:addPeer("RluMonitor", me);
	RluMonitor:connect(me,"inPosition","Localizator","outPose");
	RluMonitor:connect(me,"inCurrentTwist","Localizator","outTwist");
end



