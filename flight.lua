Propeller = {}

function Propeller:new(transmission)
    local t = setmetatable({}, { __index = Propeller })
    t.transmission = peripheral.wrap(transmission)
    t.lastPower = 0
    return t
end

function Propeller:setPower(power)
    self.transmission.setSignal(15 - power)
    self.lastPower = power
end
