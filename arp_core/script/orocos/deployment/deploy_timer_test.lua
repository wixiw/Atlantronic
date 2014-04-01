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
Reporting:setPeriod (0.002)

Deployer:loadComponent("HwTimer","OCL::TimerComponent")
assert( Deployer:setActivity("HwTimer",0.00,60,rtt.globals.ORO_SCHED_RT));
assert( Deployer:addPeer("Reporting", "HwTimer"))
HwTimer=Deployer:getPeer("HwTimer")
assert( HwTimer:configure() )
assert( HwTimer:start() )
assert( HwTimer:startTimer(0,0.010) )

-- chargement du composant periodique
assert( Deployer:loadComponent("Can1","arp_core::PeriodicClock"));
--assert( Deployer:setActivity("Can1",0.010,60,rtt.globals.ORO_SCHED_RT));
assert( Deployer:setActivity("Can1",0.00,60,rtt.globals.ORO_SCHED_RT));
assert( Deployer:addPeer("Reporting", "Can1"))
Can1=Deployer:getPeer("Can1")
--assert( Deployer:connect("Can1.inHwTimerEvent","HwTimer.timer_0",cp))
assert( Can1:configure() )
assert( Can1:start() )

-- Trace les timings
Reporting=Deployer:getPeer("Reporting")
Reporting:reportPort("Can1","outPeriod")
Reporting:reportPort("HwTimer","timeout")
Reporting:reportPort("HwTimer","timer_0")
assert( Reporting:start() )


print("fin déploiment arp_core")
print("====================")