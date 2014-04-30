dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")
dofile("/opt/ard/arp_hml/script/orocos/deployment/ubiquity/hml_monitor_deployer.lua")

HmlMonitorSimulDeployer = HmlMonitorDeployer:new()
local me = "HmlMonitor"

function HmlMonitorSimulDeployer:connect()

--connection des ports
	assert( HmlMonitorSimulDeployer:connectOneMotor("LeftDriving"))
	assert( HmlMonitorSimulDeployer:connectOneMotor("RightDriving"))
	assert( HmlMonitorSimulDeployer:connectOneMotor("RearDriving"))
	assert( HmlMonitorSimulDeployer:connectOneMotor("LeftSteering"))
	assert( HmlMonitorSimulDeployer:connectOneMotor("RightSteering"))
	assert( HmlMonitorSimulDeployer:connectOneMotor("RearSteering"))
	
	assert( Deployer:connect("HmlMonitor.inLeftSteeringHomingDone", "LeftSteering.outHomingDone",cp))
	assert( Deployer:connect("HmlMonitor.inRightSteeringHomingDone", "RightSteering.outHomingDone",cp))
	assert( Deployer:connect("HmlMonitor.inRearSteeringHomingDone", "RearSteering.outHomingDone",cp))

--ajout au monitor
	assert( HmlMonitorSimulDeployer:addToMonitor("RealTimeClock"))
	assert( HmlMonitorDeployer:addToMonitor("LeftDriving"))
	assert( HmlMonitorDeployer:addToMonitor("RightDriving"))
	assert( HmlMonitorDeployer:addToMonitor("RearDriving"))
	assert( HmlMonitorDeployer:addToMonitor("LeftSteering"))
	assert( HmlMonitorDeployer:addToMonitor("RightSteering"))
	assert( HmlMonitorDeployer:addToMonitor("RearSteering"))
	assert( HmlMonitorSimulDeployer:addToMonitor("RosHmlItf"))
	assert( HmlMonitorSimulDeployer:addToMonitor("Can1"))
	assert( HmlMonitorSimulDeployer:addToMonitor("Syncronizator"))

	assert( HmlMonitorSimulDeployer:check("HmlMonitor"))
	
	return true
end


function HmlMonitorSimulDeployer:start()
	HmlMonitor = assert( Deployer:getPeer(me))
	assert( HmlMonitor:configure())
	assert( HmlMonitor:start())
	
	return true
end