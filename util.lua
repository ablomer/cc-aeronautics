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

-- Wrap a named peripheral. Returns nil if the name is missing, empty,
-- or not attached. Unlike findPeripheral, this never errors: named
-- roles that are optional (the stabilizer) use this so a missing
-- peripheral disables that system instead of crashing startup.
function wrapOptional(name)
    if type(name) ~= "string" or name == "" then
        return nil
    end
    return peripheral.wrap(name)
end

-- Discover peripherals by type so unique-type roles do not need a
-- ComputerCraft string ID in config.lua. peripheral.find returns one
-- wrapped table per match (see https://tweaked.cc/module/peripheral.html).

local function peripheralNames(wrapped)
    local names = {}
    for i, p in ipairs(wrapped) do
        names[i] = peripheral.getName(p)
    end
    return table.concat(names, ", ")
end

-- Exactly one peripheral of this type. Errors if none or more than one.
function findPeripheral(ty)
    local found = { peripheral.find(ty) }
    if #found == 0 then
        error("Required peripheral not found: " .. ty)
    end
    if #found > 1 then
        error("Expected one " .. ty .. " but found " .. #found
            .. " (" .. peripheralNames(found) .. ")")
    end
    return found[1]
end

-- At most one peripheral of this type. Returns nil if none (so the
-- caller can disable that system). Errors if more than one.
function findOptionalPeripheral(ty)
    local found = { peripheral.find(ty) }
    if #found > 1 then
        error("Expected one " .. ty .. " but found " .. #found
            .. " (" .. peripheralNames(found) .. ")")
    end
    return found[1]
end

-- One or more peripherals of this type. Errors if none.
function findPeripherals(ty)
    local found = { peripheral.find(ty) }
    if #found == 0 then
        error("Required peripheral not found: " .. ty)
    end
    return found
end

-- Zero or more peripherals of this type. Never errors: an empty list
-- disables that system (speakers) instead of crashing startup.
function findOptionalPeripherals(ty)
    return { peripheral.find(ty) }
end
