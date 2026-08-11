Propeller = {}

function Propeller:new(transmission)
    local t = setmetatable({}, { __index = Propeller })
    t.transmission = peripheral.wrap(transmission)
    return t
end

function Propeller:setPower(power)
    self.transmission.setSignal(15 - power)
end
