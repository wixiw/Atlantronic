dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")


HmlMonitorDeployer = ComposantDeployer:new()
local me = "HmlMonitor"

function HmlMonitorDeployer:load()
	assert( Deployer:loadComponent(me, "arp_hml::HmlMonitor"))
	assert( Deployer:setActivity(me, 0.100, 10, rtt.globals.ORO_SCHED_RT))
	--turn it into a Corba server
	assert( Deployer:server(me, true))
	return true
end


function HmlMonitorDeployer:connectOneMotor(name)
	assert( Deployer:connect("HmlMonitor.in"..name.."Enable", name..".outDriveEnable",cp))
	assert( Deployer:connect("HmlMonitor.in"..name.."Connected", name..".outConnected",cp))
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


function HmlMonitorDeployer:registerToSql()
	OrocosSqlMonitor = assert( Deployer:getPeer("OrocosSqlBridge"))
	assert(Deployer:addPeer("OrocosSqlBridge",me))
	assert(OrocosSqlMonitor:ooRegisterBoolPort(me,"outDrivingEnable"))
	assert(OrocosSqlMonitor:ooRegisterBoolPort(me,"outEnable"))
	assert(OrocosSqlMonitor:ooRegisterBoolPort(me,"outSteeringEnable"))
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

	assert(Deployer:connect("HmlMonitor.inWoodheadInConnected", "WoodheadIn.outConnected",cp))
	assert(Deployer:connect("HmlMonitor.inWoodheadOutConnected", "WoodheadOut.outConnected",cp))
	
	assert(Deployer:connect("HmlMonitor.inLeftSteeringHomingDone", "LeftSteering.outHomingDone",cp))
	assert(Deployer:connect("HmlMonitor.inRightSteeringHomingDone", "RightSteering.outHomingDone",cp))
	assert(Deployer:connect("HmlMonitor.inRearSteeringHomingDone", "RearSteering.outHomingDone",cp))

	assert(Deployer:connect("HmlMonitor.inLeftDrivingBlocked", "LeftDriving.outMaxTorqueTimeout",cp))
	assert(Deployer:connect("HmlMonitor.inRightDrivingBlocked", "RightDriving.outMaxTorqueTimeout",cp))
	assert(Deployer:connect("HmlMonitor.inRearDrivingBlocked", "RearDriving.outMaxTorqueTimeout",cp))
	assert(Deployer:connect("HmlMonitor.inLeftSteeringBlocked", "LeftSteering.outMaxTorqueTimeout",cp))
	assert(Deployer:connect("HmlMonitor.inRightSteeringBlocked", "RightSteering.outMaxTorqueTimeout",cp))
	assert(Deployer:connect("HmlMonitor.inRearSteeringBlocked", "RearSteering.outMaxTorqueTimeout",cp))

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

	--assert(HmlMonitorDeployer:registerToSql())
	
	assert(HmlMonitorDeployer:check("HmlMonitor"))
	return true
end

function HmlMonitorDeployer:start()
	HmlMonitor = assert(Deployer:getPeer(me))
	assert(HmlMonitor:configure())
	assert(HmlMonitor:start())

	LeftDriving = assert(HmlMonitor:getPeer("LeftDriving"))
	RightDriving = assert(HmlMonitor:getPeer("RightDriving"))
	RearDriving = assert(HmlMonitor:getPeer("RearDriving"))
	LeftSteering = assert(HmlMonitor:getPeer("LeftSteering"))
	RightSteering = assert(HmlMonitor:getPeer("RightSteering"))
	RearSteering = assert(HmlMonitor:getPeer("RearSteering"))

	print("setting motor mode")

	assert(LeftDriving:ooSetOperationMode("other"))
	assert(RightDriving:ooSetOperationMode("other"))
	assert(RearDriving:ooSetOperationMode("other"))
	assert(LeftSteering:ooSetOperationMode("other"))
	assert(RightSteering:ooSetOperationMode("other"))
	assert(RearSteering:ooSetOperationMode("other"))

	print("setting torques")
	LeftDriving:ooFaulhaberCmd(0x81,RearDriving:getProperty("propMaximalTorque"):get()*1000)
	RightDriving:ooFaulhaberCmd(0x81,RearDriving:getProperty("propMaximalTorque"):get()*1000)
	RearDriving:ooFaulhaberCmd(0x81,RearDriving:getProperty("propMaximalTorque"):get()*1000)
	LeftSteering:ooFaulhaberCmd(0x81,RearSteering:getProperty("propMaximalTorque"):get()*1000)
	RightSteering:ooFaulhaberCmd(0x81,RearSteering:getProperty("propMaximalTorque"):get()*1000)
	RearSteering:ooFaulhaberCmd(0x81,RearSteering:getProperty("propMaximalTorque"):get()*1000)
	LeftDriving:ooFaulhaberCmd(0x80,RearDriving:getProperty("propMaximalTorque"):get()*1000)
	RightDriving:ooFaulhaberCmd(0x80,RearDriving:getProperty("propMaximalTorque"):get()*1000)
	RearDriving:ooFaulhaberCmd(0x80,RearDriving:getProperty("propMaximalTorque"):get()*1000)
	LeftSteering:ooFaulhaberCmd(0x80,RearSteering:getProperty("propMaximalTorque"):get()*1000)
	RightSteering:ooFaulhaberCmd(0x80,RearSteering:getProperty("propMaximalTorque"):get()*1000)
	RearSteering:ooFaulhaberCmd(0x80,RearSteering:getProperty("propMaximalTorque"):get()*1000)
	print("sleep")
	
	RearDriving:ooSleep(1);

	-- il n'est pas necessaire de repasser dans les bons modes de pilotage 
	-- puisqu'il faudrait faire un enable drive qui de toutes façons repassera tout le monde comme il faut

	return true
end

