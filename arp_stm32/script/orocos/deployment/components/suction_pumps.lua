dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")


SuctionPumpsDeployer = ComposantDeployer:new()
local leftFinger 	= "LeftFingerPump"
local rightFinger 	= "RightFingerPump"
local arm 			= "ArmPump"

function SuctionPumpsDeployer:loadCpnt(name, type)
	assert( Deployer:loadComponent(name,type))
	assert( Deployer:addPeer("DotGraph",name))
	assert( Deployer:setMasterSlaveActivity("Discovery", name))
end

function SuctionPumpsDeployer:load()
	SuctionPumpsDeployer:loadCpnt(leftFinger, 	"arp_stm32::SuctionPump")
	SuctionPumpsDeployer:loadCpnt(rightFinger, 	"arp_stm32::SuctionPump")
	SuctionPumpsDeployer:loadCpnt(arm, 			"arp_stm32::SuctionPump")
	return true
end

function SuctionPumpsDeployer:connectPump(name)
	assert( Deployer:stream(name..".outObjectPresent",	ros:topic("/Ubiquity/"..name.."/object_present")))
end

function SuctionPumpsDeployer:connect()
	SuctionPumpsDeployer:connectPump(leftFinger)
	SuctionPumpsDeployer:connectPump(rightFinger)
	SuctionPumpsDeployer:connectPump(arm)
	return true
end

function SuctionPumpsDeployer:start()
	--nothing to do as we are added in a monitor
	return true
end


