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


-- RPM differential a normalized steer command asks for. maxDiff is the
-- differential at full lock; at maxDiff == maxRpm the props fully
-- counter-rotate (left -maxRpm, right +maxRpm) for the fastest pivot.
function steerDifferential(steer, maxDiff)
    return maxDiff * steer
end

-- RPM left over for surge once the differential has taken its share.
-- The surge loop needs this to know the ceiling it is really working
-- against; see VelocityHold:setCeiling.
function surgeHeadroom(steer, maxRpm, maxDiff)
    local headroom = maxRpm - math.abs(steerDifferential(steer, maxDiff))
    if headroom < 0 then
        return 0
    end
    return headroom
end

-- Split a surge command (RPM) and a steer command in [-1, 1] onto
-- left/right propeller RPM, with yaw priority: the differential gets
-- first claim on the RPM budget and surge is limited to the headroom
-- that remains. That keeps the commanded turn rate the same at any
-- throttle, instead of collapsing once the outside prop saturates.
-- Clamping surge to the headroom guarantees both sides land inside
-- [-maxRpm, maxRpm], so no per-side clamp is needed afterwards.
function allocatePropMix(surge, steer, maxRpm, maxDiff)
    local diff = steerDifferential(steer, maxDiff)
    local headroom = surgeHeadroom(steer, maxRpm, maxDiff)
    surge = math.max(-headroom, math.min(headroom, surge))
    return surge - diff, surge + diff
end
