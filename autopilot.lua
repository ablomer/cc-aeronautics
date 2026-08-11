require("util")

-- VelocityHold maintains a target velocity and computes a PI control
-- output to drive it. Its read() method is compatible as a MixerChannel input.
VelocityHold = {}

VelocityHold.GAIN    = 15.0  -- proportional gain; tune as needed
VelocityHold.I_GAIN  = 2.0   -- integral gain; tune as needed
VelocityHold.I_LIMIT = 15.0  -- clamps the integral to prevent windup

function VelocityHold:new(velocitySensor, onOutput, minOutput, maxOutput)
    local t = setmetatable({}, { __index = VelocityHold })
    t.sensor   = velocitySensor
    t.target   = 0
    t.integral = ClampedIntegral:new(VelocityHold.I_LIMIT)
    t.onOutput = onOutput    -- optional callback(value) fired when a power value is computed
    t.output   = Range:new(minOutput or 0, maxOutput or VelocityHold.I_LIMIT)
    return t
end

-- Set an explicit target velocity (e.g. from a number input: hold:setTarget(10))
function VelocityHold:setTarget(velocity)
    self.target = velocity
    self.integral:reset()  -- reset integral on target change to avoid windup carry-over
end

-- Nudge the target velocity by a small amount without resetting the integral.
-- Use this for incremental adjustments (e.g. button presses) to avoid sudden
-- control changes.
function VelocityHold:nudgeTarget(delta)
    self.target = self.target + delta
end

-- Capture the current velocity as the target (used when engaging hold mode).
-- Optionally pass the current throttle to seed the integral so output starts
-- smoothly from the current power level rather than from zero.
function VelocityHold:captureTarget(currentThrottle)
    self:setTarget(self.sensor.getVelocity())
    if currentThrottle ~= nil then
        -- Back-calculate integral so initial output matches current throttle.
        -- At capture moment error is 0, so output = integral * I_GAIN.
        self.integral:set(currentThrottle / VelocityHold.I_GAIN)
    end
end

-- Returns a PI power value based on velocity error.
-- Compatible as a MixerChannel input read function.
function VelocityHold:read()
    local error = self.target - self.sensor.getVelocity()
    self.integral:add(error)
    local value = (error * VelocityHold.GAIN) + (self.integral.value * VelocityHold.I_GAIN)
    value = self.output:clamp(value)
    if self.onOutput then self.onOutput(value) end
    return value
end
