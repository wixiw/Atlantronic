dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")
dofile("/opt/ard/arp_hml/script/orocos/deployment/ubiquity/motor_deployer.lua");

MotorSimulDeployer = MotorDeployer:new()

function MotorSimulDeployer:loadMotor(name)
	Deployer:loadComponent(name,"arp_hml::MotorSimul")
	Deployer:setActivity(name,0.0,40,1)
end

function MotorSimulDeployer:load()
	--TODO remettre en aperiodique quand inCLock sera geree
	
	MotorSimulDeployer:loadMotor("LeftDriving");
	MotorSimulDeployer:loadMotor("RightDriving");
	MotorSimulDeployer:loadMotor("RearDriving");
	MotorSimulDeployer:loadMotor("LeftSteering");
	MotorSimulDeployer:loadMotor("RightSteering");
	MotorSimulDeployer:loadMotor("RearSteering");

	Deployer:loadComponent("Can1","arp_core::PeriodicClock")
	Deployer:setActivity("Can1",0.010,60,1)

end

function MotorSimulDeployer:connectMotor(name)
	Deployer:addPeer("Reporting", name)
	--MotorDeployer:registerToSql("LeftDriving")
	Deployer:connect(name..".inClock", "Can1.outClock",cp)
	MotorDeployer:check(name)
end

function MotorSimulDeployer:connect()
	MotorSimulDeployer:connectMotor("LeftDriving")
	MotorSimulDeployer:connectMotor("RightDriving")
	MotorSimulDeployer:connectMotor("RearDriving")
	MotorSimulDeployer:connectMotor("LeftSteering")
	MotorSimulDeployer:connectMotor("RightSteering")
	MotorSimulDeployer:connectMotor("RearSteering")
end



