ControlChannel = {}

function ControlChannel:new(readFn, ...)
    local t = setmetatable({}, { __index = ControlChannel })
    t.read = readFn
    t.bindings = { ... }
    return t
end

function ControlChannel:update()
    local value = self.read()
    for _, apply in ipairs(self.bindings) do
        apply(value)
    end
end

-- OnChangeOutput wraps a setter function and only calls it when the value changes.
-- Useful for physical outputs (levers, indicators) that make noise or flicker on
-- repeated identical writes.
-- An optional tolerance can be provided to suppress changes smaller than that amount.
OnChangeOutput = {}

function OnChangeOutput:new(setFn, tolerance)
    local t = setmetatable({}, { __index = OnChangeOutput })
    t.setFn = setFn
    t.tolerance = tolerance or 0
    t.lastValue = nil
    return t
end

function OnChangeOutput:set(value)
    if self.lastValue == nil or math.abs(value - self.lastValue) > self.tolerance then
        self.setFn(value)
        self.lastValue = value
    end
end
