dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")


RluMonitorDeployer = ComposantDeployer:new()
local me = "RluMonitor"

function RluMonitorDeployer:load()
	Deployer:loadComponent(me, "arp_core::Monitor")
	Deployer:setActivity(me, 0.100, 10, 1)
	--turn it into a Corba server (caution it prevent the normal deployer from deploying)
	Deployer:server(me, true)
end


function RluMonitorDeployer:addToMonitor(name)
	OdsMonitor = Deployer:getPeer(me)
	Deployer:addPeer(me, name)
	OdsMonitor:ooAddMonitoredPeer (name)
	Deployer:removePeer (name)
end


function RluMonitorDeployer:registerToSql()
	OrocosSqlMonitor = Deployer:getPeer("OrocosSqlBridge")
	Deployer:addPeer("OrocosSqlBridge",me)
end


function RluMonitorDeployer:connect()

--ajout au monitor
	RluMonitorDeployer:addToMonitor("Odometry")
	RluMonitorDeployer:addToMonitor("Localizator")
	RluMonitorDeployer:addToMonitor("LaserOnlyLocalizator")
	RluMonitorDeployer:addToMonitor("LocalizatorFilter")
	RluMonitorDeployer:addToMonitor("ObstacleManager")
	RluMonitorDeployer:addToMonitor("RosRluItf")
	--RluMonitorDeployer:registerToSql();
	
	RluMonitorDeployer:check(me)
end

function RluMonitorDeployer:start()
	OdsMonitor = Deployer:getPeer(me)
	
	OdsMonitor:configure()
	OdsMonitor:start()
end

