dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")


OdsMonitorDeployer = ComposantDeployer:new()
local me = "OdsMonitor"

function OdsMonitorDeployer:load()
	Deployer:loadComponent(me, "arp_core::Monitor")
	Deployer:setActivity(me, 0.100, 10, 1)
	--turn it into a Corba server (caution it prevent the normal deployer from deploying)
	Deployer:server(me, true)
end


function OdsMonitorDeployer:addToMonitor(name)
	OdsMonitor = Deployer:getPeer(me)
	Deployer:addPeer(me, name)
	OdsMonitor:ooAddMonitoredPeer (name)
	Deployer:removePeer (name)
end


function OdsMonitorDeployer:registerToSql()
	OrocosSqlMonitor = Deployer:getPeer("OrocosSqlBridge")
	Deployer:addPeer("OrocosSqlBridge",me)
end


function OdsMonitorDeployer:connect()

--ajout au monitor
	OdsMonitorDeployer:addToMonitor("KinematicBase")
	OdsMonitorDeployer:addToMonitor("MotionControl")
	OdsMonitorDeployer:addToMonitor("RosOdsItf")
	--OdsMonitorDeployer:registerToSql();
	
	OdsMonitorDeployer:check("OdsMonitor")
end

function OdsMonitorDeployer:start()
	OdsMonitor = Deployer:getPeer(me)
	
	OdsMonitor:configure()
	OdsMonitor:start()
end

