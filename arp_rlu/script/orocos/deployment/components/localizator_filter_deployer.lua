dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")


LocalizatorFilterDeployer = ComposantDeployer:new()
local me = "LocalizatorFilter"

function LocalizatorFilterDeployer:load()
	Deployer:loadComponent(me, "arp_rlu::LocFilterCpn")
	Deployer:setMasterSlaveActivity("MotionScheduler", me)
end


function LocalizatorFilterDeployer:registerToSql()
	OrocosSqlMonitor = Deployer:getPeer("OrocosSqlBridge")
	Deployer:addPeer("OrocosSqlBridge",me)
end


function LocalizatorFilterDeployer:connect()
	Deployer:connect(me..".inPose","Localizator.outPose",cp)
	Deployer:addPeer("Reporting", me)
	LocalizatorFilterDeployer:check(me)
end


