dofile("/opt/ard/ubiquity/src/subsystems/orocos/component_deployer_object.lua")


DiscoveryDeployer = ComposantDeployer:new()
local me = "Discovery"

function DiscoveryDeployer:load(simulation)
	if simulation == true 
	then
		assert( Deployer:loadComponent(me,"arp_stm32::Discovery"))
	else
		assert( Deployer:loadComponent(me,"arp_stm32::SimulatedDiscovery"))
	end
	
	--MasterActivity has to be set before Slaves one (else => segfault) so master activity are done here
	--TODO probleme de temps de cycle reduction de periode
	-- schedule par MainTimer
	assert( Deployer:setActivity(me,0.0,20,rtt.globals.ORO_SCHED_RT));

	return true
end

function DiscoveryDeployer:connect()
	assert( Deployer:addPeer("Reporting", me) )
	assert( Deployer:addPeer("DotGraph",me))
	assert( Deployer:connect(me..".trigger", "MainTimer.timer_1",cp ))
	return true
end

function DiscoveryDeployer:start()
	Discovery = assert(Deployer:getPeer(me))
	sched_order= assert( Discovery:getProperty("sched_order"))
	sched_order:get():resize(1)
	sched_order[0]="Gyrometer"
	
	assert(Discovery:configure())
	assert(Discovery:start())
	return true
end


