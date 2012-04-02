dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")


HmlCmdMockupDeployer = ComposantDeployer:new()
local me = "HmlCmdMockup";

function HmlCmdMockupDeployer:load()
	Deployer:loadComponent(me,"arp_hml::HmlCmdMockup")
	Deployer:setActivity(me,0.100,10,1)
end


function HmlCmdMockupDeployer:connectOneMotor(name)
	HmlMonitor = Deployer:getPeer("HmlMonitor");
	HmlMonitor:connect(name, "inSpeedCmd", me, "out"..name.."SpeedCmd")
	HmlMonitor:connect(name, "inPositionCmd", me, "out"..name.."PositionCmd")
	HmlMonitor:connect(name, "inTorqueCmd", me, "out"..name.."TorqueCmd")
end

function HmlCmdMockupDeployer:connect()
	
	Deployer:addPeer(me, "HmlMonitor")
	--on s'ajoute en peer a HmlMonitor pour pouvoir faire les connections
	Deployer:addPeer("HmlMonitor", me)

	HmlCmdMockupDeployer:connectOneMotor("LeftDriving")
	HmlCmdMockupDeployer:connectOneMotor("RightDriving")
	HmlCmdMockupDeployer:connectOneMotor("RearDriving")
	HmlCmdMockupDeployer:connectOneMotor("LeftSteering")
	HmlCmdMockupDeployer:connectOneMotor("RightSteering")
	HmlCmdMockupDeployer:connectOneMotor("RearSteering")

	HmlMonitor = Deployer:getPeer("HmlMonitor");
	HmlMonitor:connect("WoodheadOut","inBit1",me,"outBit01")
	HmlMonitor:connect("WoodheadOut","inBit2",me,"outBit02")
	HmlMonitor:connect("WoodheadOut","inBit3",me,"outBit03")
	HmlMonitor:connect("WoodheadOut","inBit4",me,"outBit04")
	HmlMonitor:connect("WoodheadOut","inBit5",me,"outBit05")
	HmlMonitor:connect("WoodheadOut","inBit6",me,"outBit06")
	HmlMonitor:connect("WoodheadOut","inBit7",me,"outBit07")
	HmlMonitor:connect("WoodheadOut","inBit8",me,"outBit08")
	
	HmlCmdMockupDeployer:check("HmlCmdMockup")
end

function HmlCmdMockupDeployer:start()
	HmlCmdMockup = Deployer:getPeer(me)
	HmlCmdMockup:configure()
	HmlCmdMockup:start()
end