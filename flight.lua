require("util")

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

-- BurnerBank fans one commanded amount out to every hot air burner in the
-- collection. Clamping/rounding happens here at the actuator boundary so
-- callers (controllers) never have to worry about the peripheral's accepted
-- range.
BurnerBank = {}

function BurnerBank:new(burners)
    local t = setmetatable({}, { __index = BurnerBank })
    t.burners = burners
    t.lastAmount = BURNER_AMOUNT_RANGE.min
    return t
end

function BurnerBank:setAmount(amount)
    amount = BURNER_AMOUNT_RANGE:clamp(amount)
    for _, burner in ipairs(self.burners) do
        burner.setTargetAmount(amount)
    end
    self.lastAmount = amount
end
