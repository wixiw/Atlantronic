dofile("/opt/ard/ubiquity/src/subsystems/orocos/component_deployer_object.lua")
dofile("/opt/ard/ubiquity/src/subsystems/stm32/components/discovery_monitor.lua");
dofile("/opt/ard/ubiquity/src/subsystems/stm32/components/simulated_discovery.lua");
dofile("/opt/ard/ubiquity/src/subsystems/stm32/components/dynamixels.lua");
dofile("/opt/ard/ubiquity/src/subsystems/stm32/components/gyrometer.lua");
dofile("/opt/ard/ubiquity/src/subsystems/stm32/components/match_data.lua");
dofile("/opt/ard/ubiquity/src/subsystems/stm32/components/suction_pumps.lua");
dofile("/opt/ard/ubiquity/src/subsystems/stm32/components/tor.lua");

Stm32SimulDeployer = ComposantDeployer:new()

function Stm32SimulDeployer:load()
	print("... LOAD stm32 simu")
	assert(Deployer:import("arp_stm32"))
	assert( MatchDataDeployer:load())
	assert( DynamixelsDeployer:load())
	assert( SuctionPumpsDeployer:load())
	assert( TorDeployer:load())
	assert( GyrometerDeployer:load())
	assert( DiscoveryMonitorDeployer:load())

	return true
end

function Stm32SimulDeployer:connect()
	print("... CONNECT stm32 simu")
	--assert( MatchDataDeployer:connect())
	assert( DynamixelsDeployer:connect())
	assert( SuctionPumpsDeployer:connect())
	assert( TorDeployer:connect())
	assert( GyrometerDeployer:connect())
	assert( DiscoveryMonitorDeployer:connect())
	return true
end


function Stm32SimulDeployer:start()
	print("... START stm32")
	assert( DiscoveryMonitorDeployer:start())
	return true
end

