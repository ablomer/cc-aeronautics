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
