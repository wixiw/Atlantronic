require("rttlib")
--require("rfsm")
--require("rfsm_rtt")

rttlib.color=true
Deployer = rtt.getTC()

-- chargement des librairies contenants les composants d'arp_hml
Deployer:import("arp_hml");

dofile("/opt/ard/arp_hml/script/orocos/deployment/ubiquity/hml_cmd_mockup_deployer.lua");

print("deploying hml Mockups")
HmlCmdMockupDeployer:load()
HmlCmdMockupDeployer:connect()
HmlCmdMockupDeployer:start()

