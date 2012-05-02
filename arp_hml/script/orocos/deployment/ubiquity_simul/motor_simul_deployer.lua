dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")
dofile("/opt/ard/arp_hml/script/orocos/deployment/ubiquity/motor_deployer.lua");

MotorSimulDeployer = MotorDeployer:new()

function MotorSimulDeployer:loadMotor(name)
	Deployer:loadComponent(name,"arp_hml::MotorSimul")
	Deployer:addPeer("MockSched", name)
	Deployer:setMasterSlaveActivity("MockSched", name)
end

function MotorSimulDeployer:load()
	Deployer:loadComponent("Can1","arp_core::PeriodicClock")
	Deployer:setActivity("Can1",0.030,60,1)
	
	Deployer:loadComponent("MockSched","FBSched")
	Deployer:setActivity("MockSched",0.0,60,1)
	MockSched = Deployer:getPeer("MockSched")
	MockSched:cleanup()
	
	MotorSimulDeployer:loadMotor("LeftDriving");
	MotorSimulDeployer:loadMotor("RightDriving");
	MotorSimulDeployer:loadMotor("RearDriving");
	MotorSimulDeployer:loadMotor("LeftSteering");
	MotorSimulDeployer:loadMotor("RightSteering");
	MotorSimulDeployer:loadMotor("RearSteering");
end

function MotorSimulDeployer:connectMotor(name)
	Deployer:addPeer("Reporting", name)
	--MotorDeployer:registerToSql("LeftDriving")
	Deployer:connect(name..".inClock", "Can1.outClock",cp)
	MotorDeployer:check(name)
end

function MotorSimulDeployer:connect()
	Deployer:addPeer("MockSched", "MotionScheduler")
	MotorSimulDeployer:connectMotor("LeftDriving")
	MotorSimulDeployer:connectMotor("RightDriving")
	MotorSimulDeployer:connectMotor("RearDriving")
	MotorSimulDeployer:connectMotor("LeftSteering")
	MotorSimulDeployer:connectMotor("RightSteering")
	MotorSimulDeployer:connectMotor("RearSteering")
	Deployer:connect("MockSched.trigger", "Can1.outTrigger",cp)
	
	MotionScheduler=Deployer:getPeer("MotionScheduler")
	sched_order=MotionScheduler:getProperty("sched_order")
	sched_order:get():resize(7)
	sched_order[0]="LeftSteering"
	sched_order[1]="RightSteering"
	sched_order[2]="RearSteering"
	sched_order[3]="LeftDriving"
	sched_order[4]="RightDriving"
	sched_order[5]="RearDriving"
	sched_order[6]="MotionScheduler"
end



