require("rttlib")

rttlib.color=true
Deployer = rtt.getTC()
print("====================")
print("début déploiment arp_stm32")

-- chargement des librairies contenants les composants d'arp_stm32
Deployer:import("arp_stm32");

dofile("/opt/ard/arp_stm32/script/orocos/deployment/components/discovery_monitor_deployer.lua");
dofile("/opt/ard/arp_stm32/script/orocos/deployment/components/discovery.lua");
dofile("/opt/ard/arp_stm32/script/orocos/deployment/components/dynamixels.lua");
dofile("/opt/ard/arp_stm32/script/orocos/deployment/components/gyrometer.lua");
dofile("/opt/ard/arp_stm32/script/orocos/deployment/components/hmi.lua");
dofile("/opt/ard/arp_stm32/script/orocos/deployment/components/match_data.lua");
dofile("/opt/ard/arp_stm32/script/orocos/deployment/components/suction_pumps.lua");
dofile("/opt/ard/arp_stm32/script/orocos/deployment/components/tor.lua");
dofile("/opt/ard/arp_stm32/script/orocos/deployment/components/hokuyo_stm32.lua");

print("... load components")
assert( DiscoveryDeployer:load(), 		"Failed to load Discovery")
assert( MatchDataDeployer:load(), 		"Failed to load MatchData")
assert( DynamixelsDeployer:load(), 		"Failed to load Dynamixels")
assert( SuctionPumpsDeployer:load(), 	"Failed to load SuctionPumps")
assert( TorDeployer:load(), 			"Failed to load Tor")
assert( GyrometerDeployer:load(), 		"Failed to load Gyrometer")
assert( HokuyoStm32Deployer:load(),"Failed to load Hokuyo")
assert( DiscoveryMonitorDeployer:load(),"Failed to load DiscoveryMonitor")

print("... connect components")
assert( DiscoveryDeployer:connect(), 	"Failed to connect Discovery")
assert( MatchDataDeployer:connect(), 	"Failed to connect MatchData")
assert( DynamixelsDeployer:connect(), 	"Failed to connect Dynamixels")
assert( SuctionPumpsDeployer:connect(), "Failed to connect SuctionPumps")
assert( TorDeployer:connect(), 			"Failed to connect Tor")
assert( GyrometerDeployer:connect(),	"Failed to connect Gyrometer")
assert( HokuyoStm32Deployer:connect(),"Failed to load Hokuyo")
assert( DiscoveryMonitorDeployer:connect(),"Failed to connect DiscoveryMonitor")

print("... start components")
assert( DiscoveryDeployer:start(),    "Failed to start Discovery")
assert( DiscoveryMonitorDeployer:start(),"Failed to start DiscoveryMonitor")

print("fin déploiment arp_stm32")
print("====================")
