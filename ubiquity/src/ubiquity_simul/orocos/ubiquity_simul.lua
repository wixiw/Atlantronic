require("rttlib")

rttlib.color=true
Deployer = rtt.getTC()
print("===============================")
print("début déploiment ubiquity_simul")

dofile("/opt/ard/ubiquity/src/subsystems/orocos/orocos.lua")
dofile("/opt/ard/ubiquity/src/subsystems/core/core.lua")
dofile("/opt/ard/ubiquity/src/subsystems/simu/simu.lua")
dofile("/opt/ard/ubiquity/src/subsystems/stm32/stm32_simul.lua")
dofile("/opt/ard/ubiquity/src/subsystems/ods/ods.lua")
dofile("/opt/ard/ubiquity/src/subsystems/rlu/rlu.lua")


print("LOADING...")
assert( OrocosDeployer:load() 			)
assert( CoreDeployer:load() 			)
assert( SimuDeployer:load()				)
assert( Stm32SimulDeployer:load()		)
assert( OdsDeployer:load()				)
assert( RluDeployer:load()	 			)


print("CONNECTING...")
assert( OrocosDeployer:connect() 			)
assert( CoreDeployer:connect()				)
assert( SimuDeployer:connect() 				)
assert( Stm32SimulDeployer:connect()		)
assert( OdsDeployer:connect("UbiquitySimul","UbiquitySimul"))
assert( RluDeployer:connect("UbiquitySimul"))

print("STARTING...")
assert( CoreDeployer:start()			)
assert( SimuDeployer:start() 			)
assert( Stm32SimulDeployer:start()		)
assert( OdsDeployer:start()				)
assert( RluDeployer:start()				)
--last one for the last component
assert( OrocosDeployer:start() 			)

print("fin déploiment ubiquity_simul")
print("=============================")
