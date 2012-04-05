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
	--on s'ajoute en peer a HmlMonitor pour pouvoir faire les connections
	Deployer:addPeer("HmlMonitor", me)
	
	HmlMonitor:connect(me, "inLeftDrivingSpeed", 		"LeftDriving","outComputedSpeed");
	HmlMonitor:connect(me, "inRightDrivingSpeed", 		"RightDriving","outComputedSpeed");
	HmlMonitor:connect(me, "inRearDrivingSpeed", 		"RearDriving","outComputedSpeed");
	HmlMonitor:connect(me, "inLeftSteeringSpeed", 		"LeftSteering","outComputedSpeed");
	HmlMonitor:connect(me, "inRightSteeringSpeed", 		"RightSteering","outComputedSpeed");
	HmlMonitor:connect(me, "inRearSteeringSpeed", 		"RearSteering","outComputedSpeed");
	
	HmlMonitor:connect(me, "inLeftDrivingPosition", 	"LeftDriving","outMeasuredPosition");
	HmlMonitor:connect(me, "inRightDrivingPosition", 	"RightDriving","outMeasuredPosition");
	HmlMonitor:connect(me, "inRearDrivingPosition", 	"RearDriving","outMeasuredPosition");
	HmlMonitor:connect(me, "inLeftSteeringPosition", 	"LeftSteering","outMeasuredPosition");
	HmlMonitor:connect(me, "inRightSteeringPosition", 	"RightSteering","outMeasuredPosition");
	HmlMonitor:connect(me, "inRearSteeringPosition", 	"RearSteering","outMeasuredPosition");
	
	Deployer:connect(me..".inParams", "UbiquityParams.outParams",cp);
	
	OdometryDeployer:check(me)
end


