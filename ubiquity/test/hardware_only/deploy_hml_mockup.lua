require("rttlib")
--require("rfsm")
--require("rfsm_rtt")

rttlib.color=true
Deployer = rtt.getTC()

-- chargement des librairies contenants les composants d'arp_hml
Deployer:import("arp_hml");

dofile("/opt/ard/arp_hml/script/orocos/deployment/ubiquity/hml_cmd_mockup_deployer.lua");

print("deploying hml Mockups")
assert( HmlCmdMockupDeployer:load(),  		"Failed to load HmlCmdMockup")
assert( HmlCmdMockupDeployer:connect(),  	"Failed to connect HmlCmdMockup")
assert( HmlCmdMockupDeployer:start(), 		"Failed to start HmlCmdMockup")

