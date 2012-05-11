dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")


MotorDeployer = ComposantDeployer:new()

function MotorDeployer:loadMotor(name)
	--sched_type 1 = RT 
	assert( Deployer:loadComponent(name,"arp_hml::Faulhaber3268Bx4"))
	assert( Deployer:setMasterSlaveActivity("Can1", name))
	return true
end

function MotorDeployer:load()
	assert( MotorDeployer:loadMotor("LeftDriving"))
	assert( MotorDeployer:loadMotor("RightDriving"))
	assert( MotorDeployer:loadMotor("RearDriving"))
	assert( MotorDeployer:loadMotor("LeftSteering"))
	assert( MotorDeployer:loadMotor("RightSteering"))
	assert( MotorDeployer:loadMotor("RearSteering"))
	return true
end

function MotorDeployer:registerToSql(name)
	OrocosSqlMonitor = assert( Deployer:getPeer("OrocosSqlBridge"))
	assert( Deployer:addPeer("OrocosSqlBridge",name))
	assert( OrocosSqlMonitor:ooRegisterBoolPort(name,"outConnected"))
	assert( OrocosSqlMonitor:ooRegisterBoolPort(name,"outDriveEnable"))
	assert( OrocosSqlMonitor:ooRegisterStringPort(name,"outCurrentOperationMode"))
	assert( OrocosSqlMonitor:ooRegisterDoublePort(name,"outSpeed"))
	assert( OrocosSqlMonitor:ooRegisterDoublePort(name,"outPosition"))
	assert( OrocosSqlMonitor:ooRegisterDoublePort(name,"outTorque"))
	return true
end

function MotorDeployer:connectMotor(name)
	--on s'enregistre en peer au composant de trace
	assert( Deployer:addPeer("Reporting", name))
	--on enregistre chez nous le controlleur Can
	assert( Deployer:addPeer(name, "Can1"))
	--assert( MotorDeployer:registerToSql(name))
	assert( MotorDeployer:check(me))
	return true
end


function MotorDeployer:connect()
	assert( MotorDeployer:connectMotor("LeftDriving"))
	assert( MotorDeployer:connectMotor("RightDriving"))
	assert( MotorDeployer:connectMotor("RearDriving"))
	assert( MotorDeployer:connectMotor("LeftSteering"))
	assert( MotorDeployer:connectMotor("RightSteering"))
	assert( MotorDeployer:connectMotor("RearSteering"))
	return true
end



