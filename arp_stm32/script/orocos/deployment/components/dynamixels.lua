dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")


DynamixelsDeployer = ComposantDeployer:new()

local dynamixelBus 			= "DynamixelBus"

local leftCannonFinger		= "LeftCannonFinger"
local rightCannonFinger 	= "RightCannonFinger"
local leftCannonStocker 	= "LeftCannonStocker"
local rightCannonStocker 	= "RightCannonStocker"

local leftFinger 			= "LeftFinger"
local rightFinger 			= "RightFinger"

--local armSlider 			= "ArmSlider"
--local armShoulder 			= "ArmShoulder"
--local armShoulderElbow 		= "ArmShoulderElbow"
--local armWristElbow 		= "ArmWristElbow"
--local armWrist 				= "ArmWrist"

function DynamixelsDeployer:loadCpnt(name, type)
	assert( Deployer:loadComponent(name,type))
	assert( Deployer:addPeer("DotGraph",name))
	assert( Deployer:setMasterSlaveActivity("Discovery", name))
end

function DynamixelsDeployer:load()
	DynamixelsDeployer:loadCpnt(	dynamixelBus, 			"arp_stm32::DynamixelBus")
	
	DynamixelsDeployer:loadCpnt(	leftCannonFinger, 		"arp_stm32::Dynamixel")
	DynamixelsDeployer:loadCpnt(	rightCannonFinger, 		"arp_stm32::Dynamixel")
	DynamixelsDeployer:loadCpnt(	leftCannonStocker, 		"arp_stm32::Dynamixel")
	DynamixelsDeployer:loadCpnt(	rightCannonStocker, 	"arp_stm32::Dynamixel")
	
	DynamixelsDeployer:loadCpnt(	leftFinger, 			"arp_stm32::Dynamixel")
	DynamixelsDeployer:loadCpnt(	rightFinger, 			"arp_stm32::Dynamixel")
	
--	DynamixelsDeployer:loadCpnt(	armSlider, 				"arp_stm32::Dynamixel")
--	DynamixelsDeployer:loadCpnt(	armShoulder, 			"arp_stm32::Dynamixel")
--	DynamixelsDeployer:loadCpnt(	armShoulderElbow, 		"arp_stm32::Dynamixel")
--	DynamixelsDeployer:loadCpnt(	armWristElbow, 			"arp_stm32::Dynamixel")
--	DynamixelsDeployer:loadCpnt(	armWrist,				"arp_stm32::Dynamixel")
	return true
end

function DynamixelsDeployer:connectDynamixel(name)
        assert( Deployer:stream(name..".outState",      ros:topic("/Ubiquity/"..name.."/state")))
        assert( Deployer:stream(name..".inPositionCmd",      ros:topic("/Ubiquity/"..name.."/position_cmd")))
        assert( Deployer:stream(name..".inMaxTorqueAllowed",      ros:topic("/Ubiquity/"..name.."/max_torque")))
end

function DynamixelsDeployer:connect()
	assert( Deployer:addPeer("Reporting", dynamixelBus) )
	
	DynamixelsDeployer:connectDynamixel(leftCannonFinger)
	DynamixelsDeployer:connectDynamixel(rightCannonFinger)
	DynamixelsDeployer:connectDynamixel(leftCannonStocker)
	DynamixelsDeployer:connectDynamixel(rightCannonStocker)
	
	DynamixelsDeployer:connectDynamixel(leftFinger)
	DynamixelsDeployer:connectDynamixel(rightFinger)
	
--	DynamixelsDeployer:connectDynamixel(armSlider)
--	DynamixelsDeployer:connectDynamixel(armShoulder)
--	DynamixelsDeployer:connectDynamixel(armShoulderElbow)
--	DynamixelsDeployer:connectDynamixel(armWristElbow)
--	DynamixelsDeployer:connectDynamixel(armWrist)

	return true
end

function DynamixelsDeployer:start()
	--nothing to do as we are added to a monitor
	return true
end


