require("rttlib")
rttlib.color=true
Deployer = rtt.getTC()
print("====================")
print("début déploiment arp_hml")

-- chargement des librairies contenants les composants d'arp_hml
Deployer:import("arp_hml");

dofile("/opt/ard/arp_hml/script/orocos/deployment/ubiquity/joystick_deployer.lua");
dofile("/opt/ard/arp_hml/script/orocos/deployment/ubiquity_simul/hml_monitor_simul_deployer.lua");
dofile("/opt/ard/arp_hml/script/orocos/deployment/ubiquity_simul/ros_hml_itf_simul_deployer.lua");

-- chargement de l'interface HML
print("... load components")
JoystickDeployer:load()
RosHmlItfDeployer:load()
HmlMonitorSimulDeployer:load()


print("... connect components")
JoystickDeployer:connect()
RosHmlItfDeployer:connect()
HmlMonitorSimulDeployer:connect()

print("... start components")
HmlMonitorSimulDeployer:start()


print("fin déploiment arp_hml")
print("====================")
