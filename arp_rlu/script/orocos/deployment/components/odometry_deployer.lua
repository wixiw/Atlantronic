dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")


OdometryDeployer = ComposantDeployer:new()
local me = "Odometry"

function OdometryDeployer:load()
	Deployer:loadComponent(me, "arp_rlu::Odometry4Ubiquity")
	Deployer:setMasterSlaveActivity("MotionScheduler", me)
end


function OdometryDeployer:registerToSql()
	OrocosSqlMonitor = Deployer:getPeer("OrocosSqlBridge")
	Deployer:addPeer("OrocosSqlBridge",me)
end


function OdometryDeployer:connect()
	--on s'ajoute en peer a HmlMonitor pour pouvoir faire les connections
	Deployer:addPeer("HmlMonitor", me)
	Deployer:addPeer("Reporting", me)
	HmlMonitor:connect(me, "inTime", 		"Syncronizator","outClock");
	HmlMonitor:connect(me, "inMotorState", 		"Syncronizator","outMotorMeasures");
	Deployer:connect(me..".inParams", "UbiquityParams.outParams",cp);
	
	OdometryDeployer:check(me)
end


