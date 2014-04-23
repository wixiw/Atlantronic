dofile("/opt/ard/arp_core/script/orocos/deployment/component_deployer_object.lua")


TorDeployer = ComposantDeployer:new()
local leftFingerFireOmron 			= "LeftFingerFireOmron"
local rightFingerFireOmron 			= "RightFingerFireOmron"
local armFireOmron 					= "ArmFireOmron"
local leftFingerLateralOmron 		= "LeftFingerLateralOmron"
local rightFingerLateralOmron 		= "RightFingerLateralOmron"
local leftRacalOmron 				= "LeftRecalOmron"
local rightRecalOmron 				= "RightRecalOmron"

function TorDeployer:loadCpnt(name, type)
	assert( Deployer:loadComponent(name,type))
	assert( Deployer:addPeer("DotGraph",name))
	assert( Deployer:setMasterSlaveActivity("Discovery", name))
end

function TorDeployer:load()
	TorDeployer:loadCpnt(leftFingerFireOmron, 		"arp_stm32::Tor")
	TorDeployer:loadCpnt(rightFingerFireOmron, 		"arp_stm32::Tor")
	TorDeployer:loadCpnt(armFireOmron, 				"arp_stm32::Tor")
	TorDeployer:loadCpnt(leftFingerLateralOmron, 	"arp_stm32::Tor")
	TorDeployer:loadCpnt(rightFingerLateralOmron, 	"arp_stm32::Tor")
	TorDeployer:loadCpnt(leftRacalOmron, 			"arp_stm32::Tor")
	TorDeployer:loadCpnt(rightRecalOmron, 			"arp_stm32::Tor")
	return true
end

function TorDeployer:connectTor(name)
	assert( Deployer:stream(name..".outObjectPresent",	ros:topic("/Ubiquity/"..name.."/object_present")))
end

function TorDeployer:connect()
	TorDeployer:connectTor(leftFingerFireOmron)
	TorDeployer:connectTor(rightFingerFireOmron)
	TorDeployer:connectTor(armFireOmron)
	TorDeployer:connectTor(leftFingerLateralOmron)
	TorDeployer:connectTor(rightFingerLateralOmron)
	TorDeployer:connectTor(leftRacalOmron)
	TorDeployer:connectTor(rightRecalOmron)
	return true
end

function TorDeployer:start()
	LeftFingerFireOmron 		= assert(Deployer:getPeer(leftFingerFireOmron))
	RightFingerFireOmron	 	= assert(Deployer:getPeer(rightFingerFireOmron))
	ArmFireOmron 				= assert(Deployer:getPeer(armFireOmron))
	LeftFingerLateralOmron 		= assert(Deployer:getPeer(leftFingerLateralOmron))
	RightFingerLateralOmron 	= assert(Deployer:getPeer(rightFingerLateralOmron))
	LeftRacalOmron 				= assert(Deployer:getPeer(leftRacalOmron))
	RightRecalOmron 			= assert(Deployer:getPeer(rightRecalOmron))
	
	assert(LeftFingerFireOmron:configure())
	assert(RightFingerFireOmron:configure())
	assert(ArmFireOmron:configure())
	assert(LeftFingerLateralOmron:configure())
	assert(RightFingerLateralOmron:configure())
	assert(LeftRacalOmron:configure())
	assert(RightRecalOmron:configure())
	
	assert(LeftFingerFireOmron:start())
	assert(RightFingerFireOmron:start())
	assert(ArmFireOmron:start())
	assert(LeftFingerLateralOmron:start())
	assert(RightFingerLateralOmron:start())
	assert(LeftRacalOmron:start())
	assert(RightRecalOmron:start())
	return true
end


