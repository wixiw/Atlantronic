dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")


LaserOnlyLocalizatorDeployer = ComposantDeployer:new()
local me = "LaserOnlyLocalizator"

function LaserOnlyLocalizatorDeployer:load()
	Deployer:loadComponent(me, "arp_rlu::LaserOnlyLocalizatorCpn")
	Deployer:setActivity(me,0.0,0,rtt.globals.ORO_SCHED_OTHER)
end


function LaserOnlyLocalizatorDeployer:registerToSql()
	OrocosSqlMonitor = Deployer:getPeer("OrocosSqlBridge")
	Deployer:addPeer("OrocosSqlBridge",me)
end


function LaserOnlyLocalizatorDeployer:connect()
	Deployer:stream(me..".inScan",ros:topic("/top_scan"))
	Deployer:addPeer("Reporting", me)
	LaserOnlyLocalizatorDeployer:check(me)
end


