require("rttlib")
rttlib.color=true
Deployer = rtt.getTC()
print("====================")
print("début déploiment arp_hml")

-- chargement des librairies contenants les composants d'arp_hml
Deployer:import("arp_hml");


dofile("script/orocos/deployment/ubiquity/can_deployer.lua");
dofile("script/orocos/deployment/ubiquity/motor_deployer.lua");
dofile("script/orocos/deployment/ubiquity/io_deployer.lua");
dofile("script/orocos/deployment/ubiquity/hml_itf_deployer.lua");

CanDeployer:load()
MotorDeployer:load()
IoDeployer:load()
HmlItfDeployer:load()


CanDeployer:connect()
MotorDeployer:connect()
IoDeployer:connect()
HmlItfDeployer:connect()

CanDeployer:start()
MotorDeployer:start()
IoDeployer:start()
HmlItfDeployer:start()

print("fin déploiment arp_hml")
print("====================")
