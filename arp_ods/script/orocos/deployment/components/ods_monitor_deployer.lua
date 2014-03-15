dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")


OdsMonitorDeployer = ComposantDeployer:new()
local me = "OdsMonitor"

function OdsMonitorDeployer:load()
	Deployer:loadComponent(me, "arp_core::Monitor")
	assert( Deployer:addPeer("DotGraph",me))
	Deployer:setActivity(me, 0.100, 10, 1)
end


function OdsMonitorDeployer:addToMonitor(name)
	OdsMonitor = Deployer:getPeer(me)
	Deployer:addPeer(me, name)
	OdsMonitor:ooAddMonitoredPeer (name)
	Deployer:removePeer (name)
end


function OdsMonitorDeployer:connect()
	OdsMonitorDeployer:check("OdsMonitor")
end

function OdsMonitorDeployer:start()
	OdsMonitor = Deployer:getPeer(me)
	
	OdsMonitor:configure()
	OdsMonitor:start()
end

