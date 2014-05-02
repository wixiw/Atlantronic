dofile("/opt/ard/ubiquity/src/subsystems/orocos/component_deployer_object.lua")
dofile("/opt/ard/ubiquity/src/subsystems/orocos/telemetry.lua");
dofile("/opt/ard/ubiquity/src/subsystems/orocos/last_component.lua");

OrocosDeployer = ComposantDeployer:new()

function OrocosDeployer:load()
	print("... LOAD orocos")
	assert(Deployer:import("arp_master"))
	assert(Deployer:import("rtt_dot_service"))
	
	assert(Deployer:loadComponent("Reporting","OCL::FileReporting"))
	
	assert(Deployer:loadComponent("DotGraph","TaskContext"))
	assert(Deployer:loadService("DotGraph","dot"))
	
	assert(LastComponentDeployer:load())
	return true
end

function OrocosDeployer:connect()
	print("... CONNECT orocos")
	
	assert( Deployer:setMasterSlaveActivity("MotionScheduler", "Reporting"))
	assert( LastComponentDeployer:connect())
	
	--a activer pour avoir des traces dans reports.dat (soit dans le répertoire courant soit dans /opt/ros)
	assert( Telemetry:report())
	return true
end


function OrocosDeployer:start()
	print("... START orocos")
	
	Reporting= assert(Deployer:getPeer("Reporting")) 
	propFile=Reporting:getProperty("ReportFile")
	assert(propFile:set("/tmp/reports.dat"))
	assert( Reporting:start() )
	
	DotGraph = Deployer:getPeer("DotGraph")
	assert(Deployer:getPeer("DotGraph")) 
	assert( DotGraph:configure())
	-- a patch is requiered on orocos 2.5 version. It should be present in 2.7
	--assert(assert(DotGraph:provides("dot")):getProperty("dot_file"):set("/tmp/ard_graph.dot"))
	--assert(assert(DotGraph:provides("dot")):getProperty("comp_args"):set("style=filled,width=10,height=3.5,"))
	--assert(assert(DotGraph:provides("dot")):getProperty("conn_args"):set(""))
	--assert(assert(DotGraph:provides("dot")):getProperty("chan_args"):set(""))
	
	assert(LastComponentDeployer:start())
	return true
end
