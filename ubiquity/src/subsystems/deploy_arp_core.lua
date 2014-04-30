require "rttlib"
rttlib.color=true
Deployer = rtt.getTC()
print("====================")
print("début déploiment arp_core")

-- chargement des librairies contenants les composants d'arp_core
Deployer:import("arp_core");
Deployer:import("rtt_rosnode");
Deployer:import("rtt_arp_msgs");

--chargement du generateur de visualisation des composant
print("loading DotGraph (rtt_dot_service) component...")
assert(Deployer:loadComponent("DotGraph","TaskContext"))
DotGraph = Deployer:getPeer("DotGraph")
assert(Deployer:import("rtt_dot_service"))
assert(Deployer:loadService("DotGraph","dot"))
-- a patch is requiered on orocos 2.5 version. It should be present in 2.7
--assert(assert(DotGraph:provides("dot")):getProperty("dot_file"):set("/tmp/ard_graph.dot"))
--assert(assert(DotGraph:provides("dot")):getProperty("comp_args"):set("style=filled,width=10,height=3.5,"))
--assert(assert(DotGraph:provides("dot")):getProperty("conn_args"):set(""))
--assert(assert(DotGraph:provides("dot")):getProperty("chan_args"):set(""))
assert( DotGraph:configure(), "Failed to configure DotGraph")

-- chargement du composant de reporting
print("loading Reporting component...")
assert(Deployer:loadComponent("Reporting","OCL::FileReporting"))
Reporting = Deployer:getPeer("Reporting")
propFile=Reporting:getProperty("ReportFile")
assert(propFile:set("/tmp/reports.dat"))
assert(Reporting:setPeriod (0.010))

-- chargement du composant serveur parametre
print("loading Ubiquity param server...")
assert( Deployer:loadComponent("UbiquityParams","arp_core::ParamsComponent"), "Failed to load UbiquityParams")
assert(Deployer:addPeer("DotGraph","UbiquityParams"))
UbiquityParams = Deployer:getPeer("UbiquityParams")
assert( UbiquityParams:setPeriod (0.100))
assert( UbiquityParams:configure(), "Failed to configure UbiquityParams")
assert( UbiquityParams:start(), "Failed to start UbiquityParams")

print("fin déploiment arp_core")
print("====================")
