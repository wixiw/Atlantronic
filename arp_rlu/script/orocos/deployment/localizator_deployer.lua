dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")


LocalizatorDeployer = ComposantDeployer:new()
local me = "Localizator"

function LocalizatorDeployer:load()
	Deployer:loadComponent(me, "arp_rlu::Localizator")
	Deployer:setMasterSlaveActivity("MotionScheduler", me)
	
	Deployer:loadComponent("LocFilter", "arp_rlu::LocFilterCpn")
	Deployer:setMasterSlaveActivity("MotionScheduler", "LocFilter")
end


function LocalizatorDeployer:registerToSql()
	OrocosSqlMonitor = Deployer:getPeer("OrocosSqlBridge")
	Deployer:addPeer("OrocosSqlBridge",me)
end


function LocalizatorDeployer:connect()
	Deployer:connect(me..".inOdo","Odometry.outTwist",cp)
	Deployer:stream(me..".inScan",ros:topic("/top_scan"))
	Deployer:addPeer("Reporting", me)
	
	Deployer:connect("LocFilter.inPose","Localizator.outPose",cp)
	Deployer:addPeer("Reporting", "LocFilter")
	LocalizatorDeployer:check(me)
end




