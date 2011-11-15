require("rttlib")
rttlib.color=true
Deployer = rtt.getTC()
print("====================")
print("début déploiment arp_hml")

-- chargement des librairies contenants les composants d'arp_hml
Deployer:import("arp_hml");


dofile("script/orocos/deployment/ubiquity/can_deployer.lua");
dofile("script/orocos/deployment/ubiquity/io_deployer.lua");

print("... load components")
CanDeployer:load()
IoDeployer:load()

print("... connect components")
CanDeployer:connect()
IoDeployer:connect()

print("... start components")
CanDeployer:start()
IoDeployer:start()

print("fin déploiment arp_hml")
print("====================")
