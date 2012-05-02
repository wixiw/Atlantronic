dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")


LocalizatorDeployer = ComposantDeployer:new()
local me = "Localizator"

function LocalizatorDeployer:load()
	Deployer:loadComponent(me, "arp_rlu::Localizator")
	Deployer:setMasterSlaveActivity("MotionScheduler", me)
end


function LocalizatorDeployer:registerToSql()
	OrocosSqlMonitor = Deployer:getPeer("OrocosSqlBridge")
	Deployer:addPeer("OrocosSqlBridge",me)
end


function LocalizatorDeployer:connect()
	Deployer:connect(me..".inOdo","Odometry.outTwist",cp)
	Deployer:stream(me..".inScan",ros:topic("/top_scan"))
	Deployer:addPeer("Reporting", me)
	LocalizatorDeployer:check(me)
end


