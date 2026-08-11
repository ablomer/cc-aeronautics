-- VelocityHold maintains a target velocity and computes a proportional
-- power output to drive it. Its read() method makes it compatible as a
-- ControlChannel readFn.
VelocityHold = {}

VelocityHold.GAIN = 1.0  -- proportional gain; tune as needed

function VelocityHold:new(velocitySensor, onOutput)
    local t = setmetatable({}, { __index = VelocityHold })
    t.sensor = velocitySensor
    t.target = 0
    t.onOutput = onOutput  -- optional callback(value) fired when a power value is computed
    return t
end

-- Set an explicit target velocity (e.g. from a number input: hold:setTarget(10))
function VelocityHold:setTarget(velocity)
    self.target = velocity
end

-- Capture the current velocity as the target (used when engaging hold mode)
function VelocityHold:captureTarget()
    self:setTarget(self.sensor.getVelocity())
end

-- Returns a proportional power value based on velocity error.
-- Compatible as a ControlChannel readFn.
function VelocityHold:read()
    local value = (self.target - self.sensor.getVelocity()) * VelocityHold.GAIN
    if self.onOutput then self.onOutput(value) end
    return value
end
