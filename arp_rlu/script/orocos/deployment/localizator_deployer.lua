dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")


LocalizatorDeployer = ComposantDeployer:new()
local me = "Localizator"

function LocalizatorDeployer:load()
	Deployer:loadComponent(me, "arp_rlu::Localizator")
	Deployer:setActivity(me, 0.0, 10, 1)
end


function LocalizatorDeployer:registerToSql()
	OrocosSqlMonitor = Deployer:getPeer("OrocosSqlBridge")
	Deployer:addPeer("OrocosSqlBridge",me)
end


function LocalizatorDeployer:connect()
	Deployer:connect(me..".inOdo","Odometry.outTwist",cp)
	
	LocalizatorDeployer:check(me)
end


