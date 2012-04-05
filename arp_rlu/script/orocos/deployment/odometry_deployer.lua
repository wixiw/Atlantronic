dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")


OdometryDeployer = ComposantDeployer:new()
local me = "Odometry"

function OdometryDeployer:load()
	Deployer:loadComponent(me, "arp_rlu::Odometry")
	Deployer:setActivity(me, 0.100, 10, 1)
end


function OdometryDeployer:registerToSql()
	OrocosSqlMonitor = Deployer:getPeer("OrocosSqlBridge")
	Deployer:addPeer("OrocosSqlBridge",me)
end


function OdometryDeployer:connect()

	
	OdometryDeployer:check(me)
end


