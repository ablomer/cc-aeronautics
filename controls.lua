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
OnChangeOutput = {}

function OnChangeOutput:new(setFn)
    local t = setmetatable({}, { __index = OnChangeOutput })
    t.setFn = setFn
    t.lastValue = nil
    return t
end

function OnChangeOutput:set(value)
    if value ~= self.lastValue then
        self.setFn(value)
        self.lastValue = value
    end
end
