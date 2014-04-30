dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")
dofile("/opt/ard/arp_hml/script/orocos/deployment/ubiquity/motor_deployer.lua");

MotorSimulDeployer = MotorDeployer:new()

function MotorSimulDeployer:loadMotor(name)
	assert( Deployer:loadComponent(name,"arp_simu::MotorSimul"))
	assert( Deployer:addPeer("DotGraph",name))
	assert( Deployer:addPeer("MotionScheduler", name))
	assert( Deployer:setMasterSlaveActivity("MotionScheduler", name))
	return true
end

function MotorSimulDeployer:load()
	assert( MotorSimulDeployer:loadMotor("LeftDriving"));
	assert( MotorSimulDeployer:loadMotor("RightDriving"));
	assert( MotorSimulDeployer:loadMotor("RearDriving"));
	assert( MotorSimulDeployer:loadMotor("LeftSteering"));
	assert( MotorSimulDeployer:loadMotor("RightSteering"));
	assert( MotorSimulDeployer:loadMotor("RearSteering"));
	
	return true
end

function MotorSimulDeployer:connectMotor(name)
	assert( Deployer:addPeer("Reporting", name))
	assert( Deployer:connect(name..".inClock", "RealTimeClock.outClock",cp))
	assert( Deployer:connect(name..".inBlockMotor", "Can1.outWheelBlocked",cp))
	assert( MotorSimulDeployer:check(name))
	return true
end

function MotorSimulDeployer:connect()
	assert( MotorSimulDeployer:connectMotor("LeftDriving"))
	assert( MotorSimulDeployer:connectMotor("RightDriving"))
	assert( MotorSimulDeployer:connectMotor("RearDriving"))
	assert( MotorSimulDeployer:connectMotor("LeftSteering"))
	assert( MotorSimulDeployer:connectMotor("RightSteering"))
	assert( MotorSimulDeployer:connectMotor("RearSteering"))
	
	return true
end



