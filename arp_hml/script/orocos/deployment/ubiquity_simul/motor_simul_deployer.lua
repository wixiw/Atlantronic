dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")
dofile("/opt/ard/arp_hml/script/orocos/deployment/ubiquity/motor_deployer.lua");

MotorSimulDeployer = MotorDeployer:new()

function MotorSimulDeployer:loadMotor(name)
	assert( Deployer:loadComponent(name,"arp_hml::MotorSimul"))
	assert( Deployer:addPeer("MockSched", name))
	assert( Deployer:setMasterSlaveActivity("MockSched", name))
	return true
end

function MotorSimulDeployer:load()
	assert( Deployer:loadComponent("Can1","arp_core::PeriodicClock"));
	assert( Deployer:setActivity("Can1",0.030,60,1));
	
	assert( Deployer:loadComponent("MockSched","FBSched"));
	assert( Deployer:setActivity("MockSched",0.0,60,1));
	MockSched = assert( Deployer:getPeer("MockSched"));
	assert( MockSched:cleanup());
	
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
	--assert( MotorDeployer:registerToSql("LeftDriving"))
	assert( Deployer:connect(name..".inClock", "Can1.outClock",cp))
	assert( MotorSimulDeployer:check(name))
	return true
end

function MotorSimulDeployer:connect()
	assert( Deployer:addPeer("MockSched", "MotionScheduler"))
	assert( MotorSimulDeployer:connectMotor("LeftDriving"))
	assert( MotorSimulDeployer:connectMotor("RightDriving"))
	assert( MotorSimulDeployer:connectMotor("RearDriving"))
	assert( MotorSimulDeployer:connectMotor("LeftSteering"))
	assert( MotorSimulDeployer:connectMotor("RightSteering"))
	assert( MotorSimulDeployer:connectMotor("RearSteering"))
	assert( Deployer:connect("MockSched.trigger", "Can1.outTrigger",cp))
	
	MotionScheduler=assert( Deployer:getPeer("MotionScheduler"))
	sched_order= assert( MotionScheduler:getProperty("sched_order"))
	sched_order:get():resize(7)
	sched_order[0]="LeftSteering"
	sched_order[1]="RightSteering"
	sched_order[2]="RearSteering"
	sched_order[3]="LeftDriving"
	sched_order[4]="RightDriving"
	sched_order[5]="RearDriving"
	sched_order[6]="MotionScheduler"
	
	return true
end



