require "rttlib"
rttlib.color=true
Deployer = rtt.getTC()
print("====================")
print("début déploiment arp_core")

-- chargement des librairies contenants les composants d'arp_core
Deployer:import("arp_core");
Deployer:import("rtt_rosnode");

-- chargement du composant de reporting
print("loading Reporting component...")
Deployer:loadComponent("Reporting","OCL::FileReporting")
Reporting = Deployer:getPeer("Reporting")
Reporting:setPeriod (0.050)

-- chargement du composant serveur parametre
print("loading Ubiquity param server...")
Deployer:loadComponent("UbiquityParams","arp_core::ParamsComponent")
Deployer:setActivity("UbiquityParams", 0.100, 0, 0)
UbiquityParams = Deployer:getPeer("UbiquityParams")
UbiquityParams:configure()
UbiquityParams:start();

--chargement du generateur de visualisation des composant
print("loading rtt_dot_service...")
Deployer:import("rtt_dot_service")
Deployer:loadService("Deployer","dot")
--print(Deployer:getProperty("dot.comp_args"))
--:set("style=filled,width=5,height=3.5")
--print(Deployer:provides("dot"))
print("fin déploiment arp_core")
print("====================")
