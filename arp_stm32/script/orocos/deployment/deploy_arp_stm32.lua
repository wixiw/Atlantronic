require("rttlib")

rttlib.color=true
Deployer = rtt.getTC()
print("====================")
print("début déploiment arp_stm32")

-- chargement des librairies contenants les composants d'arp_stm32
Deployer:import("arp_stm32");

dofile("/opt/ard/arp_stm32/script/orocos/deployment/components/discovery.lua");
dofile("/opt/ard/arp_stm32/script/orocos/deployment/components/match_data.lua");
dofile("/opt/ard/arp_stm32/script/orocos/deployment/components/hmi.lua");

print("... load components")
assert( DiscoveryDeployer:load(), 		"Failed to load Discovery")
assert( MatchDataDeployer:load(), 		"Failed to load MatchData")
assert( HmiDeployer:load(), 		        "Failed to load Hmi")

print("... connect components")
assert( DiscoveryDeployer:connect(), 	"Failed to connect Discovery")
assert( MatchDataDeployer:connect(), 	"Failed to connect MatchData")
assert( HmiDeployer:connect(), 	        "Failed to connect Hmi")

print("... start components")
assert( DiscoveryDeployer:start(), 		"Failed to start Discovery")
assert( MatchDataDeployer:start(), 		"Failed to start MatchData")
assert( HmiDeployer:start(), 		        "Failed to start Hmi")

print("fin déploiment arp_stm32")
print("====================")
