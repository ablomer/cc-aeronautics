-- OnChangeOutput wraps a setter function and only calls it when the value changes.
-- Useful for physical outputs (levers, indicators) that make noise or flicker on
-- repeated identical writes.
-- An optional tolerance can be provided to suppress changes smaller than that amount.
OnChangeOutput = {}

function OnChangeOutput:new(setFn, tolerance, round)
    local t = setmetatable({}, { __index = OnChangeOutput })
    t.setFn = setFn
    t.tolerance = tolerance or 0
    t.round = round or false
    t.lastValue = nil
    return t
end

function OnChangeOutput:set(value)
    local v = self.round and math.floor(value + 0.5) or value
    if self.lastValue == nil or math.abs(v - self.lastValue) > self.tolerance then
        self.setFn(v)
        self.lastValue = v
    end
end

function OnChangeOutput:invalidate()
    self.lastValue = nil
end


-- Each input is a table: { read = fn, weight = number }
-- weight defaults to 1.0 if omitted.
-- The final value is the sum of (input.read() * input.weight), clamped to [min, max].
-- Use a negative weight to invert an input (e.g. left propeller steering differential).
MixerChannel = {}

MixerChannel.DEFAULT_MIN = 0
MixerChannel.DEFAULT_MAX = 15

function MixerChannel:new(setFn, min, max, ...)
    local t = setmetatable({}, { __index = MixerChannel })
    t.setFn = setFn
    t.min    = min or MixerChannel.DEFAULT_MIN
    t.max    = max or MixerChannel.DEFAULT_MAX
    t.inputs = { ... }  -- each: { read = fn, weight = number }
    return t
end

function MixerChannel:update()
    local value = 0
    for _, input in ipairs(self.inputs) do
        value = value + (input.read() * (input.weight or 1.0))
    end
    value = math.max(self.min, math.min(self.max, value))
    self.setFn(value)
end

-- Split a surge command and a steer command in [-1, 1] onto left/right
-- propeller RPM. maxDiff is the largest half-differential (maxRpm / 2).
--
-- Cruise (pivot == false): shrink the differential so both sides stay in
-- [0, maxRpm] and the mean stays at surge. Turns do not sag speed, and
-- a flying prop is never reversed.
-- Pivot (pivot == true): clamp independently in [-maxRpm, maxRpm] so the
-- stopped hull can yaw with opposite rotation. Speed hold is parked at 0.
function allocatePropMix(surge, steer, maxRpm, maxDiff, pivot)
    if pivot then
        local right = surge + maxDiff * steer
        local left  = surge - maxDiff * steer
        right = math.max(-maxRpm, math.min(maxRpm, right))
        left  = math.max(-maxRpm, math.min(maxRpm, left))
        return left, right
    end
    local maxD = math.min(surge, maxRpm - surge, maxDiff)
    if maxD < 0 then maxD = 0 end
    local right = surge + maxD * steer
    local left  = surge - maxD * steer
    return left, right
end
