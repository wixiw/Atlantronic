dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")

KinematicBaseDeployer = ComposantDeployer:new()
local me = "KinematicBase"

function KinematicBaseDeployer:load()
	Deployer:loadComponent(me,"arp_ods::KinematicBase");
	Deployer:setActivity(me,0.0,26,1);
end


function KinematicBaseDeployer:registerToSql()
	OrocosSqlMonitor = Deployer:getPeer("OrocosSqlBridge")
	Deployer:addPeer("OrocosSqlBridge",me)
end

function KinematicBaseDeployer:connect()
	HmlMonitor = Deployer:getPeer("HmlMonitor");
	--on s'ajoute en peer a HmlMonitor pour pouvoir faire les connections
	Deployer:addPeer("HmlMonitor", me)
	HmlMonitor:connect(me,"inMotorState","Syncronizator","outMotorMeasures");
	Deployer:connect(me..".inParams","UbiquityParams.outParams",cp);
	Deployer:connect(me..".inTwistCmd","MotionControl.outTwistCmd",cp);
	HmlMonitor:connect("LeftDriving","inSpeedCmd",me,"outLeftDrivingVelocityCmd");
	HmlMonitor:connect("RightDriving","inSpeedCmd",me,"outRightDrivingVelocityCmd");
	HmlMonitor:connect("RearDriving","inSpeedCmd",me,"outRearDrivingVelocityCmd");
	HmlMonitor:connect("LeftSteering","inPositionCmd",me,"outLeftSteeringPositionCmd");
	HmlMonitor:connect("RightSteering","inPositionCmd",me,"outRightSteeringPositionCmd");
	HmlMonitor:connect("RearSteering","inPositionCmd",me,"outRearSteeringPositionCmd");
	
	RluMonitor = Deployer:getPeer("RluMonitor");
	--on s'ajoute en peer a HmlMonitor pour pouvoir faire les connections
	Deployer:addPeer("RluMonitor", me)
	RluMonitor:connect(me,"inCurrentTwist","Localizator","outTwist");
end



