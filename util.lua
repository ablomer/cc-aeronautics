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
