dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")


HmlMonitorDeployer = ComposantDeployer:new()
local me = "HmlMonitor"

function HmlMonitorDeployer:load()
	assert( Deployer:loadComponent(me, "arp_hml::HmlMonitor"))
	assert( Deployer:setActivity(me, 0.100, 0, rtt.globals.ORO_SCHED_OTHER))
	return true
end


function HmlMonitorDeployer:connectOneMotor(name)
	assert( Deployer:connect("HmlMonitor.in"..name.."Enable", name..".outDriveEnable",cp))
	assert(Deployer:connect("HmlMonitor.in"..name.."Blocked", name..".outMaxTorqueTimeout",cp))
	return true
end


function HmlMonitorDeployer:addToMonitor(name)
	--print("... adding " ..name.. " to HmlMonitor")
	HmlMonitor = assert( Deployer:getPeer(me))
	assert( Deployer:addPeer(me, name))
	assert( HmlMonitor:ooAddMonitoredPeer (name))
	Deployer:removePeer (name)
	return true
end

function HmlMonitorDeployer:addToBusMonitor(name)
	--print("... adding " ..name.. " BUS to HmlMonitor")
	HmlMonitor = assert( Deployer:getPeer(me))
	assert( Deployer:addPeer(me, name))
	assert( HmlMonitor:ooAddHmlBusMonitoredPeer(name))
	Deployer:removePeer (name)
	return true
end


function HmlMonitorDeployer:connect()

--connection des ports
	assert(HmlMonitorDeployer:connectOneMotor("LeftDriving"))
	assert(HmlMonitorDeployer:connectOneMotor("RightDriving"))
	assert(HmlMonitorDeployer:connectOneMotor("RearDriving"))
	assert(HmlMonitorDeployer:connectOneMotor("LeftSteering"))
	assert(HmlMonitorDeployer:connectOneMotor("RightSteering"))
	assert(HmlMonitorDeployer:connectOneMotor("RearSteering"))
	
	assert(Deployer:connect("HmlMonitor.inLeftSteeringHomingDone", "LeftSteering.outHomingDone",cp))
	assert(Deployer:connect("HmlMonitor.inRightSteeringHomingDone", "RightSteering.outHomingDone",cp))
	assert(Deployer:connect("HmlMonitor.inRearSteeringHomingDone", "RearSteering.outHomingDone",cp))
	
	assert(Deployer:connect("HmlMonitor.inCanBusConnected", "Can1.outBusConnected",cp))


--ajout au monitor

	assert(HmlMonitorDeployer:addToBusMonitor("Can1"))

	assert(HmlMonitorDeployer:addToMonitor("LeftDriving"))
	assert(HmlMonitorDeployer:addToMonitor("RightDriving"))
	assert(HmlMonitorDeployer:addToMonitor("RearDriving"))
	assert(HmlMonitorDeployer:addToMonitor("LeftSteering"))
	assert(HmlMonitorDeployer:addToMonitor("RightSteering"))
	assert(HmlMonitorDeployer:addToMonitor("RearSteering"))

	assert(HmlMonitorDeployer:addToMonitor("WoodheadIn"))
	assert(HmlMonitorDeployer:addToMonitor("WoodheadOut"))
	assert(HmlMonitorDeployer:addToMonitor("Joystick"))
	assert(HmlMonitorDeployer:addToMonitor("Syncronizator"))

	assert(HmlMonitorDeployer:addToMonitor("RosHmlItf"))
	
	assert(HmlMonitorDeployer:check("HmlMonitor"))
	return true
end

function HmlMonitorDeployer:start()
	HmlMonitor = assert(Deployer:getPeer(me))
	assert(HmlMonitor:configure())
	assert(HmlMonitor:start())

--	Can1 = assert(HmlMonitor:getPeer("Can1"))
--	LeftDriving = assert(HmlMonitor:getPeer("LeftDriving"))
--	RightDriving = assert(HmlMonitor:getPeer("RightDriving"))
--	RearDriving = assert(HmlMonitor:getPeer("RearDriving"))
--	LeftSteering = assert(HmlMonitor:getPeer("LeftSteering"))
--	RightSteering = assert(HmlMonitor:getPeer("RightSteering"))
--	RearSteering = assert(HmlMonitor:getPeer("RearSteering"))
--	WoodheadIn = assert(HmlMonitor:getPeer("WoodheadIn"))
--	WoodheadOut = assert(HmlMonitor:getPeer("WoodheadOut"))

--	print("configuring CanBus")
--	Can1:ooConfigureDevices()
--	print("starting CanBus")
--	Can1:ooConfigureDevices()

--	print("setting torques")
--	LeftDriving:ooLimitCurrent(RearDriving:getProperty("propMaximalTorque"):get())
--	RightDriving:ooLimitCurrent(RearDriving:getProperty("propMaximalTorque"):get())
--	RearDriving:ooLimitCurrent(RearDriving:getProperty("propMaximalTorque"):get())
--	LeftSteering:ooLimitCurrent(RearSteering:getProperty("propMaximalTorque"):get())
--	RightSteering:ooLimitCurrent(RearSteering:getProperty("propMaximalTorque"):get())
--	RearSteering:ooLimitCurrent(RearSteering:getProperty("propMaximalTorque"):get())

	-- il n'est pas necessaire de repasser dans les bons modes de pilotage 
	-- puisqu'il faudrait faire un enable drive qui de toutes façons repassera tout le monde comme il faut

	return true
end

