dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")


HmlCmdMockupDeployer = ComposantDeployer:new()
local me = "HmlCmdMockup";

function HmlCmdMockupDeployer:load()
	Deployer:loadComponent(me,"arp_hml::HmlCmdMockup")
	Deployer:setActivity(me,0.100,10,1)
end


function HmlCmdMockupDeployer:connectOneMotor(name)
	Deployer:connect(name..".inSpeedCmd",		me..".out"..name.."SpeedCmd",cp)
	Deployer:connect(name..".inPositionCmd",	me..".out"..name.."PositionCmd",cp)
	Deployer:connect(name..".inTorqueCmd",		me..".out"..name.."TorqueCmd",cp)
end

function HmlCmdMockupDeployer:connect()
	
	Deployer:addPeer("RosHmlItf", "HmlMonitor")

--connection des ports
	--HmlCmdMockupDeployer:connectOneMotor("LeftDriving")
	--HmlCmdMockupDeployer:connectOneMotor("RightDriving")
	HmlCmdMockupDeployer:connectOneMotor("RearDriving")
	--HmlCmdMockupDeployer:connectOneMotor("LeftSteering")
	--HmlCmdMockupDeployer:connectOneMotor("RightSteering")
	HmlCmdMockupDeployer:connectOneMotor("RearSteering")

	Deployer:connect("WoodheadOut.inBit1",me..".outBit01",cp)
	Deployer:connect("WoodheadOut.inBit2",me..".outBit02",cp)
	Deployer:connect("WoodheadOut.inBit3",me..".outBit03",cp)
	Deployer:connect("WoodheadOut.inBit4",me..".outBit04",cp)
	Deployer:connect("WoodheadOut.inBit5",me..".outBit05",cp)
	Deployer:connect("WoodheadOut.inBit6",me..".outBit06",cp)
	Deployer:connect("WoodheadOut.inBit7",me..".outBit07",cp)
	Deployer:connect("WoodheadOut.inBit8",me..".outBit08",cp)
	
	--HmlCmdMockupDeployer:check("HmlCmdMockup")
end

function HmlCmdMockupDeployer:start()
	HmlCmdMockup = Deployer:getPeer(me)
	HmlCmdMockup:configure()
	HmlCmdMockup:provides("ioOutSelfTest"):activate()
	HmlCmdMockup:provides("ioOutSelfTest"):automatic()
	
	HmlCmdMockup:start()
end