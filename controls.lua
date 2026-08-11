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

-- MixerChannel combines N weighted input channels into a single output.
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
