dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")

TwistTeleopDeployer = ComposantDeployer:new()
local me = "MotionControl"

function TwistTeleopDeployer:load()
	Deployer:loadComponent(me,"arp_ods::TwistTeleop");
	Deployer:setActivity(me,0.050,25,1);
end


function TwistTeleopDeployer:registerToSql()
	OrocosSqlMonitor = Deployer:getPeer("OrocosSqlBridge")
	Deployer:addPeer("OrocosSqlBridge",me)
end

function TwistTeleopDeployer:connect()
	MotionControl = Deployer:getPeer(me);
	Deployer:addPeer("HmlMonitor", me)
	
	HmlMonitor:connect(me,"inXSpeed","Joystick","outX1");
	HmlMonitor:connect(me,"inYSpeed","Joystick","outY1");
	HmlMonitor:connect(me,"inThetaSpeed","Joystick","outX2");
end





