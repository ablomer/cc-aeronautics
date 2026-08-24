-- Range represents a [min, max] interval and provides clamping.
Range = {}

function Range:new(min, max)
    local t = setmetatable({}, { __index = Range })
    t.min = min
    t.max = max
    return t
end

function Range:clamp(value)
    return math.max(self.min, math.min(self.max, value))
end

-- ClampedIntegral accumulates a value while keeping it within [-limit, limit].
ClampedIntegral = {}

function ClampedIntegral:new(limit)
    local t = setmetatable({}, { __index = ClampedIntegral })
    t.range = Range:new(-limit, limit)
    t.value = 0
    return t
end

function ClampedIntegral:add(amount)
    self.value = self.range:clamp(self.value + amount)
end

function ClampedIntegral:set(value)
    self.value = self.range:clamp(value)
end

function ClampedIntegral:reset()
    self.value = 0
end

-- Advance a controller's lastClock and return a sane dt (seconds).
-- First tick or a non-positive delta uses defaultDt. Stalls longer than
-- maxDt are capped so a hitch does not dump a huge integral/slew step.
function stepClock(state, defaultDt, maxDt)
    maxDt = maxDt or 1.0
    local now = os.clock()
    local dt = defaultDt
    if state.lastClock ~= nil then
        dt = now - state.lastClock
        if dt <= 0 then dt = defaultDt end
        if dt > maxDt then dt = maxDt end
    end
    state.lastClock = now
    return dt
end

-- Shared burner amount range, used by both the BurnerBank actuator (flight.lua)
-- and the VerticalSpeedHold controller (autopilot.lua) so the controller's
-- clamping/anti-windup math always agrees with what the actuator accepts.
BURNER_AMOUNT_RANGE = Range:new(5, 500)

-- True for a real number (rejects nil, NaN, and non-numbers).
function isFiniteNumber(v)
    return type(v) == "number" and v == v
end
