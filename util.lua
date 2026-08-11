-- ClampedIntegral accumulates a value while keeping it within [-limit, limit].
ClampedIntegral = {}

function ClampedIntegral:new(limit)
    local t = setmetatable({}, { __index = ClampedIntegral })
    t.limit = limit
    t.value = 0
    return t
end

function ClampedIntegral:add(amount)
    self.value = math.max(-self.limit, math.min(self.limit, self.value + amount))
end

function ClampedIntegral:set(value)
    self.value = math.max(-self.limit, math.min(self.limit, value))
end

function ClampedIntegral:reset()
    self.value = 0
end
