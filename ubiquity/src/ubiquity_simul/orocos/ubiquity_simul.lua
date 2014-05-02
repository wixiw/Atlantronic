require("rttlib")

rttlib.color=true
local SIMULATION=true
Deployer = rtt.getTC()
print("===============================")
print("début déploiment ubiquity_simul")

dofile("/opt/ard/ubiquity/src/subsystems/orocos/orocos.lua")
dofile("/opt/ard/ubiquity/src/subsystems/core/core.lua")
dofile("/opt/ard/ubiquity/src/subsystems/simu/simu.lua")
dofile("/opt/ard/ubiquity/src/subsystems/stm32/stm32_simul.lua")
dofile("/opt/ard/ubiquity/src/subsystems/ods/ods.lua")
dofile("/opt/ard/ubiquity/src/subsystems/rlu/rlu.lua")

function sleep(n)
  os.execute("sleep " .. tonumber(n))
end

print("LOADING...")
assert( OrocosDeployer:load() 					)
assert( CoreDeployer:load(SIMULATION) 			)
assert( SimuDeployer:load()						)
assert( Stm32SimulDeployer:load(SIMULATION)		)
assert( OdsDeployer:load()						)
assert( RluDeployer:load()	 					)


print("CONNECTING...")
assert( CoreDeployer:connect()				)
assert( SimuDeployer:connect() 				)
assert( Stm32SimulDeployer:connect()		)
assert( OdsDeployer:connect("UbiquitySimul","UbiquitySimul"))
assert( RluDeployer:connect("UbiquitySimul"))
assert( OrocosDeployer:connect() 			)

print("STARTING...")
--last one for the last component
assert( SimuDeployer:start() 				)
assert( RluDeployer:start()					)
assert( OdsDeployer:start()					)
assert( Stm32SimulDeployer:start()			)
assert( CoreDeployer:start()				)
assert( OrocosDeployer:start() 				)

print("fin déploiment ubiquity_simul")
print("=============================")
