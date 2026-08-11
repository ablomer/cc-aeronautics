require("util")

-- BearingHold steers the ship towards a navigation target by computing the
-- bearing error and producing a PI steering output in [-1, 1].
-- Its read() method is compatible as a MixerChannel input.
BearingHold = {}

BearingHold.GAIN       = 2 / 180   -- proportional gain; maps ±90° error to ±1.0 output
BearingHold.I_GAIN     = 2 / 180   -- integral gain; tune as needed
BearingHold.I_LIMIT    = 1.0       -- clamps the integral to prevent windup
BearingHold.MIN_OUTPUT = 0.1       -- minimum effective output magnitude to overcome drag
BearingHold.DEADBAND   = 2.0       -- degrees; bearing errors within this range are ignored

function BearingHold:new(navTable)
    local t = setmetatable({}, { __index = BearingHold })
    t.navTable  = navTable
    t.integral  = ClampedIntegral:new(BearingHold.I_LIMIT)
    t.lastBearing = nil
    return t
end

-- Seed the integral from the current bearing so the first output matches the
-- current steering state and avoids a sudden jump on engagement.
function BearingHold:captureState()
    if not self.navTable.hasTarget() then return end
    local bearing = self.navTable.getBearing()
    bearing = ((bearing + 180) % 360) - 180
    -- Back-calculate integral so initial output matches the proportional term.
    -- At capture moment: output = P + I = bearing*GAIN + integral*I_GAIN
    -- We want total = bearing*GAIN, so seed integral to 0.
    self.integral:reset()
end

-- Returns a PI steering value in [-1, 1] based on bearing error, or 0 if no target.
-- Compatible as a MixerChannel input read function.
function BearingHold:read()
    if not self.navTable.hasTarget() then
        self.lastBearing = nil
        self.lastOutput  = nil
        self.integral:reset()
        return 0
    end
    local bearing = self.navTable.getBearing()
    -- Wrap to [-180, 180] so the ship always turns the short way
    bearing = ((bearing + 180) % 360) - 180
    self.lastBearing = bearing
    -- Within deadband: output zero and let the integral drain
    if math.abs(bearing) <= BearingHold.DEADBAND then
        self.lastOutput = 0
        return 0
    end
    self.integral:add(bearing)
    local value = (bearing * BearingHold.GAIN) + (self.integral.value * BearingHold.I_GAIN)
    value = Range:new(-1, 1):clamp(value)
    -- Apply minimum output floor to overcome drag at small errors
    if value > 0 and value < BearingHold.MIN_OUTPUT then
        value = BearingHold.MIN_OUTPUT
    elseif value < 0 and value > -BearingHold.MIN_OUTPUT then
        value = -BearingHold.MIN_OUTPUT
    end
    self.lastOutput = value
    return value
end

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
