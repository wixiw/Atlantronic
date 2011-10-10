require("rttlib")
rttlib.color=true

depl = rtt.getTC()

depl:import("ocl")
depl:import("arp_core")
depl:import("arp_hml")

rtt.logl('Warning', "====================")
rtt.logl('Warning', "début déploiment arp_hml")

depl:loadComponent("Hml","arp_hml::ProtokrotItf");
Hhml = depl:getPeer("Hml")
depl:setActivity("Hml",0.050,0,ORO_SCHED_OTHER);
Hml:configure();
Hml:start();
