require "rttlib"
rttlib.color=true
Deployer = rtt.getTC()
print("====================")
print("début déploiment arp_core")

-- chargement des librairies contenants les composants d'arp_core
Deployer:import("arp_core");
Deployer:import("rtt_rosnode");
Deployer:import("rtt_arp_core");

-- chargement du composant de reporting
print("loading Reporting component...")
Deployer:loadComponent("Reporting","OCL::FileReporting")
Reporting = Deployer:getPeer("Reporting")
propFile=Reporting:getProperty("ReportFile")
propFile:set("/tmp/reports.dat")
Reporting:setPeriod (0.010)

-- chargement du composant serveur parametre
print("loading Ubiquity param server...")
assert( Deployer:loadComponent("UbiquityParams","arp_core::ParamsComponent"), "Failed to load UbiquityParams")
UbiquityParams = Deployer:getPeer("UbiquityParams")
UbiquityParams:setPeriod (0.100)
assert( UbiquityParams:configure(), "Failed to configure UbiquityParams")
assert( UbiquityParams:start(), "Failed to start UbiquityParams")

--chargement du generateur de visualisation des composant
print("loading rtt_dot_service...")
Deployer:import("rtt_dot_service")
Deployer:loadService("Deployer","dot")
--print(Deployer:getProperty("dot.comp_args"))
--:set("style=filled,width=5,height=3.5")
--print(Deployer:provides("dot"))
print("fin déploiment arp_core")
print("====================")
