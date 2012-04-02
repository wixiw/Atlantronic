-- Each component taht you want to deploy should create a new instance of ComposantDeployer
-- This instance is a kind of interface you has to implement yourself for your composant
-- In the top deployer you will use something like this :
-- Comp1=ComposantDeployer:new{ name = "truc" }
-- Comp1:load()
-- Comp1:connect()

cp=rtt.Variable("ConnPolicy")
ros=rtt.provides("ros")

ComposantDeployer = 
{
	name="noname"
}

function ComposantDeployer:new (o)
      o = o or {}
      setmetatable(o, self)
      self.__index = self
      return o
end

function ComposantDeployer:load ()
	print("loading " .. self.name)
end

function ComposantDeployer:connect ()
	print("connecting " .. self.name)
end

function ComposantDeployer:start ()
	print("starting " .. self.name)
end

-- Use this function to check if ports are conencted
function ComposantDeployer:check(tc)
   --[[
   local portnames = tc:getPortNames()
   local ret = true
   for _,pn in ipairs(portnames) do
      local p = tc:getPort(pn)
      local info = p:info()
      if info.porttype == 'in' and info.connected == false then
         rtt.logl('Error', "InputPort " .. tc:getName() .. "." .. info.name .. " is unconnected!")
         ret = false
      end
   end
   return ret
   ]]
   
end
