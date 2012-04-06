dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")


MotorDeployer = ComposantDeployer:new()

function MotorDeployer:load()
	Deployer:loadComponent("LeftDriving","arp_hml::Faulhaber3268Bx4")
	Deployer:setActivity("LeftDriving",0,40,1)
	Deployer:loadComponent("RightDriving","arp_hml::Faulhaber3268Bx4")
	Deployer:setActivity("RightDriving",0,40,1)
	Deployer:loadComponent("RearDriving","arp_hml::Faulhaber3268Bx4")
	Deployer:setActivity("RearDriving",0,40,1)

	Deployer:loadComponent("LeftSteering","arp_hml::Faulhaber3268Bx4")
	Deployer:setActivity("LeftSteering",0,40,1)
	Deployer:loadComponent("RightSteering","arp_hml::Faulhaber3268Bx4")
	Deployer:setActivity("RightSteering",0,40,1)
	Deployer:loadComponent("RearSteering","arp_hml::Faulhaber3268Bx4")
	Deployer:setActivity("RearSteering",0,40,1)
end

function MotorDeployer:registerToSql(name)
	OrocosSqlMonitor = Deployer:getPeer("OrocosSqlBridge")
	Deployer:addPeer("OrocosSqlBridge",name)
	OrocosSqlMonitor:ooRegisterBoolPort(name,"outConnected")
	OrocosSqlMonitor:ooRegisterBoolPort(name,"outDriveEnable")
	OrocosSqlMonitor:ooRegisterStringPort(name,"outCurrentOperationMode")
	OrocosSqlMonitor:ooRegisterDoublePort(name,"outComputedSpeed")
	OrocosSqlMonitor:ooRegisterDoublePort(name,"outMeasuredPosition")
	OrocosSqlMonitor:ooRegisterDoublePort(name,"outMeasuredTorque")
end

function MotorDeployer:connectMotor(name)
	--on s'enregistre en peer au composant de trace
	--on enregistre chez nous le controlleur Can
	Deployer:addPeer(me, "Can1")
	Deployer:addPeer("Reporting", me)
	--MotorDeployer:registerToSql("LeftDriving")
	Deployer:connect(name..".inClock", "Can1.outClock",cp)
	MotorDeployer:check(me)
end


function MotorDeployer:connect()
	MotorDeployer:connectMotor("LeftDriving")
	MotorDeployer:connectMotor("RightDriving")
	MotorDeployer:connectMotor("RearDriving")
	MotorDeployer:connectMotor("LeftSteering")
	MotorDeployer:connectMotor("RightSteering")
	MotorDeployer:connectMotor("RearSteering")
end



