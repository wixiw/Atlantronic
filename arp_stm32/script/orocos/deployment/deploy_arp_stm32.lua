require("rttlib")

rttlib.color=true
Deployer = rtt.getTC()
print("====================")
print("début déploiment arp_stm32")

-- chargement des librairies contenants les composants d'arp_stm32
Deployer:import("arp_stm32");

dofile("/opt/ard/arp_stm32/script/orocos/deployment/discovery/discovery.lua");

print("... load components")
assert( DiscoveryDeployer:load(), 		"Failed to load Discovery")

print("... connect components")
assert( DiscoveryDeployer:connect(), 			"Failed to connect Discovery")

print("... start components")
assert( DiscoveryDeployer:start(), "Failed to start Discovery")

print("fin déploiment arp_stm32")
print("====================")
