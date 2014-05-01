dofile("/opt/ard/ubiquity/src/subsystems/orocos/component_deployer_object.lua")
dofile("/opt/ard/ubiquity/src/subsystems/simu/components/ros_hml_simu_itf.lua");
--dofile("/opt/ard/ubiquity/src/subsystems/simu/components/hml_monitor.lua");
dofile("/opt/ard/ubiquity/src/subsystems/simu/components/ubiquity_simulation.lua");
dofile("/opt/ard/ubiquity/src/subsystems/simu/components/scheduler.lua");
--dofile("/opt/ard/ubiquity/src/subsystems/simu/components/motor_simul.lua");
--dofile("/opt/ard/ubiquity/src/subsystems/simu/components/syncronizator.lua");

SimuDeployer = ComposantDeployer:new()

function SimuDeployer:load()
	print("... LOAD simu")
	assert(Deployer:import("arp_simu"))
	assert( SimulatedDiscoveryDeployer:load())
	assert( UbiquitySimulDeployer:load())
	assert( SchedulerDeployer:load())
	--assert( MotorSimulDeployer:load())
	--assert( Syncronizator:load())
	assert( RosHmlSimuItfDeployer:load())
	--assert( HmlMonitorSimulDeployer:load())
	return true
end

function SimuDeployer:connect()
	print("... CONNECT simu")
	assert( SimulatedDiscoveryDeployer:connect())
	assert( UbiquitySimulDeployer:connect())
	assert( SchedulerDeployer:connect())
	--assert( MotorSimulDeployer:connect())
	--assert( Syncronizator:connect())
	assert( RosHmlSimuItfDeployer:connect())
	--assert( HmlMonitorSimulDeployer:connect())
	return true
end


function SimuDeployer:start()
	print("... START simu")
	assert( SimulatedDiscoveryDeployer:start())
	--assert( HmlMonitorSimulDeployer:start());
		--TODO bug opengl
	--assert( HmiDeployer:start())
	return true
end

