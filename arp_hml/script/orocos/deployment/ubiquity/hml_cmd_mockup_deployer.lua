dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")


HmlCmdMockupDeployer = ComposantDeployer:new()
local me = "HmlCmdMockup";

function HmlCmdMockupDeployer:load()
	assert( Deployer:loadComponent(me,"arp_hml::HmlCmdMockup"))
	assert( Deployer:setActivity(me,0.100,0,rtt.globals.ORO_SCHED_OTHER))
	return true
end


function HmlCmdMockupDeployer:connectOneMotor(name)
	assert( HmlMonitor = Deployer:getPeer("HmlMonitor"))
	assert( HmlMonitor:connect(name, "inSpeedCmd", me, "out"..name.."SpeedCmd"))
	assert( HmlMonitor:connect(name, "inPositionCmd", me, "out"..name.."PositionCmd"))
	assert( HmlMonitor:connect(name, "inTorqueCmd", me, "out"..name.."TorqueCmd"))
	return true
end

function HmlCmdMockupDeployer:connect()
	
	assert( Deployer:addPeer(me, "HmlMonitor"))
	--on s'ajoute en peer a HmlMonitor pour pouvoir faire les connections
	assert( Deployer:addPeer("HmlMonitor", me))

	assert( HmlCmdMockupDeployer:connectOneMotor("LeftDriving"))
	assert( HmlCmdMockupDeployer:connectOneMotor("RightDriving"))
	assert( HmlCmdMockupDeployer:connectOneMotor("RearDriving"))
	assert( HmlCmdMockupDeployer:connectOneMotor("LeftSteering"))
	assert( HmlCmdMockupDeployer:connectOneMotor("RightSteering"))
	assert( HmlCmdMockupDeployer:connectOneMotor("RearSteering"))

	HmlMonitor = assert( Deployer:getPeer("HmlMonitor"))
	assert( HmlMonitor:connect("WoodheadOut","inBit1",me,"outBit01"))
	assert( HmlMonitor:connect("WoodheadOut","inBit2",me,"outBit02"))
	assert( HmlMonitor:connect("WoodheadOut","inBit3",me,"outBit03"))
	assert( HmlMonitor:connect("WoodheadOut","inBit4",me,"outBit04"))
	assert( HmlMonitor:connect("WoodheadOut","inBit5",me,"outBit05"))
	assert( HmlMonitor:connect("WoodheadOut","inBit6",me,"outBit06"))
	assert( HmlMonitor:connect("WoodheadOut","inBit7",me,"outBit07"))
	assert( HmlMonitor:connect("WoodheadOut","inBit8",me,"outBit08"))
	
	assert( HmlCmdMockupDeployer:check("HmlCmdMockup"))
	return true
end

function HmlCmdMockupDeployer:start()
	HmlCmdMockup = Deployer:getPeer(me)
	HmlCmdMockup:configure()
	HmlCmdMockup:start()
	return true
end