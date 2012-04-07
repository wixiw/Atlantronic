dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")


MotorDeployer = ComposantDeployer:new()

function MotorDeployer:loadMotor(name)
	--sched_type 1 = RT 
	Deployer:loadComponent(name,"arp_hml::Faulhaber3268Bx4")
	Deployer:setActivity(name,0,40,1)
end

function MotorDeployer:load()
	MotorDeployer:loadMotor("LeftDriving");
	MotorDeployer:loadMotor("RightDriving");
	MotorDeployer:loadMotor("RearDriving");
	MotorDeployer:loadMotor("LeftSteering");
	MotorDeployer:loadMotor("RightSteering");
	MotorDeployer:loadMotor("RearSteering");
end

function MotorDeployer:registerToSql(name)
	OrocosSqlMonitor = Deployer:getPeer("OrocosSqlBridge")
	Deployer:addPeer("OrocosSqlBridge",name)
	OrocosSqlMonitor:ooRegisterBoolPort(name,"outConnected")
	OrocosSqlMonitor:ooRegisterBoolPort(name,"outDriveEnable")
	OrocosSqlMonitor:ooRegisterStringPort(name,"outCurrentOperationMode")
	OrocosSqlMonitor:ooRegisterDoublePort(name,"outSpeed")
	OrocosSqlMonitor:ooRegisterDoublePort(name,"outPosition")
	OrocosSqlMonitor:ooRegisterDoublePort(name,"outTorque")
end

function MotorDeployer:connectMotor(name)
	--on s'enregistre en peer au composant de trace
	Deployer:addPeer("Reporting", name)
	--on enregistre chez nous le controlleur Can
	Deployer:addPeer(name, "Can1")
	--MotorDeployer:registerToSql(name)
	MotorDeployer:check(me)
end


function MotorDeployer:connect()
	MotorDeployer:connectMotor("LeftDriving")
	MotorDeployer:connectMotor("RightDriving")
	MotorDeployer:connectMotor("RearDriving")
	MotorDeployer:connectMotor("LeftSteering")
	MotorDeployer:connectMotor("RightSteering")
	MotorDeployer:connectMotor("RearSteering")
	
	Deployer:addPeer("LeftDriving", "Can1");
	Deployer:addPeer("Reporting", "LeftDriving")
	Deployer:addPeer("RightDriving", "Can1");
	Deployer:addPeer("Reporting", "RightDriving")
	Deployer:addPeer("RearDriving", "Can1");
	Deployer:addPeer("Reporting", "RearDriving")
end



