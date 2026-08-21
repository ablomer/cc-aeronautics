require("autopilot")

-- GroundProtection computes a minimum-safe altitude target from the ship's
-- current height and the closest optical ground-clearance reading, so that
-- AltitudeHold's own outer loop (altitude error -> desired vertical speed)
-- naturally demands enough climb to avoid descending faster than is safe
-- for the remaining clearance.
--
-- Design intent: guard against descending too fast for the available
-- clearance, not against merely being close to the ground. Sitting stably
-- at low altitude produces no correction; only a target that would demand
-- an unsafe descent rate (given current clearance) gets overridden. This
-- keeps protection compatible with a future deliberate landing mode, which
-- will operate inside this same envelope at low speed rather than fighting
-- it.
--
-- This works by raising the ALTITUDE TARGET fed into AltitudeHold, rather
-- than injecting a vertical-speed floor into the middle of its cascade. The
-- existing outer loop, inner PI, integral, and slew logic are reused
-- unchanged -- there is exactly one target and one control law, and only
-- one place (main.lua) ever calls burnerBank:setAmount.
--
-- This module drives the burner side of ground protection (via the target
-- correction below) using AltitudeHold's much slower response. The bottom
-- lift-propeller bank (liftpropeller.lua) is the fast-responding half of
-- the same system: it reads GroundProtection.allowedDescentRate() directly
-- rather than going through a target correction, since it isn't cascaded
-- through AltitudeHold at all. Both actuators always agree on what "too
-- fast" means for a given clearance because both consult this one table.
-- LandingSequence (landing.lua) is a third, independent consumer: it
-- proposes its own descent target, which is combined with this module's
-- floor via math.max() in main.lua, so a landing request can never
-- override the floor computed here.
--
-- CLEARANCE_TABLE is an unverified placeholder and must be tuned in-game
-- against the ship's actual climb authority and momentum before being
-- trusted -- these numbers are starting points, not measured limits.
GroundProtection = {}

-- Each point is { distance = metres (0-15), rate = m/s }, sorted ascending
-- by distance. rate is the most-negative (most-descending) vertical speed
-- considered safe at that clearance; positive values mean "must already be
-- climbing". The topmost entry's rate matches -AltitudeHold.MAX_DESCENT_RATE
-- so that at full clearance (15 m, the optical sensor's own confirmed
-- "clear" reading) this system adds no restriction beyond what AltitudeHold
-- already enforces on its own.
-- FIXED: distance=0 used to require rate=+3.0 ("must already be climbing").
-- That directly contradicted this module's own stated design intent above
-- ("sitting stably ... produces no correction") and would have made a
-- resting touchdown -- distance ~0, verticalSpeed ~0 -- look like an active
-- hazard forever, with the propellers fighting the ground indefinitely and
-- landing never able to complete. distance=0 must cap at rate=0 (may not
-- still be descending, but resting there is fine) for landing to work at
-- all; every other entry stays at or below 0 for the same reason.
GroundProtection.CLEARANCE_TABLE = {
    { distance = 0,  rate =  0.0 },  -- ground contact: must not still be descending; resting (0) is fine
    { distance = 2,  rate = -0.3 },
    { distance = 5,  rate = -0.8 },
    { distance = 9,  rate = -1.4 },
    { distance = 15, rate = -2.0 },  -- == -AltitudeHold.MAX_DESCENT_RATE: no added restriction
}

local function isFiniteNumber(v)
    return type(v) == "number" and v == v and v ~= math.huge and v ~= -math.huge
end

-- Linear interpolation through CLEARANCE_TABLE. Distances outside the
-- table's range clamp to the nearest endpoint. Exposed as a function on the
-- class (rather than kept local) so other actuators -- currently
-- LiftPropellerHold (liftpropeller.lua) -- can share this exact envelope
-- instead of maintaining a second, possibly inconsistent one.
function GroundProtection.allowedDescentRate(distance)
    local points = GroundProtection.CLEARANCE_TABLE
    if distance <= points[1].distance then
        return points[1].rate
    end
    for i = 2, #points do
        local a, b = points[i - 1], points[i]
        if distance <= b.distance then
            local frac = (distance - a.distance) / (b.distance - a.distance)
            return a.rate + (b.rate - a.rate) * frac
        end
    end
    return points[#points].rate
end

-- Inverts AltitudeHold's own outer-loop formula (deadband-shrunk altitude
-- error * APPROACH_GAIN = desired rate) to find the MINIMUM target
-- altitude at which AltitudeHold's outer loop would demand AT LEAST `rate`
-- from `height`. "Minimum" matters here, not "exact": this is meant to be
-- combined with other targets via math.max() (both by computeFloorTarget
-- below and by landing.lua), so overshooting it costs real authority.
-- Expressing the correction in AltitudeHold's own terms means no second
-- control law is introduced anywhere this is used.
--
-- Exposed as a class function (like allowedDescentRate above) so
-- landing.lua can request a target for a rate that's a fraction of the
-- safety envelope, rather than reimplementing this inversion itself.
--
-- FIXED: this used to split on rate >= 0, which put rate == 0 in the
-- "climbing" branch (height + DEADBAND + 0 = height + 3). desiredRate == 0
-- is produced by AltitudeHold's entire deadband interval
-- [height-DEADBAND, height+DEADBAND], not a single point, and the minimum
-- target in that interval satisfying "rate >= 0" is the LOWER edge
-- (height - DEADBAND), not the upper one. The old code therefore demanded
-- a spurious 3 m climb at distance == 0 (rate == 0) -- exactly the
-- ground-contact/touchdown case -- while distance slightly above 0 (rate
-- slightly negative) correctly floored near height - 3. That discontinuity
-- would have fought a real landing right at the moment of touchdown.
-- Splitting on rate > 0 (strictly) routes rate == 0 into the same branch
-- as negative rates, which already evaluates to height - DEADBAND when
-- shrunkError is 0 -- the correct minimum for "must not still be
-- descending, resting is fine".
function GroundProtection.targetForRate(height, rate)
    local shrunkError = rate / AltitudeHold.APPROACH_GAIN
    if rate > 0 then
        return height + AltitudeHold.DEADBAND + shrunkError
    else
        return height - AltitudeHold.DEADBAND + shrunkError
    end
end

function GroundProtection:new()
    local t = setmetatable({}, { __index = GroundProtection })
    t.lastAllowedRate = nil
    t.lastFloorTarget = nil  -- nil means no floor was applied on the last call
    return t
end

-- Returns a floor altitude target, or -math.huge if no correction applies
-- this tick (unknown/invalid height or distance, or clearance not
-- restrictive beyond AltitudeHold's own normal descent limit). Callers
-- should combine this with the pilot's requested target via math.max().
--
-- height: current altitude reading. Validated independently of
-- AltitudeHold's own fault handling -- if this reading is bad,
-- AltitudeHold:read() will freeze the burner amount regardless of target
-- on this tick anyway, so no correction is needed or possible here.
-- distance: closest optical clearance reading, already normalized to 0-15
-- by OpticalSensorBank (a sensor fault reports 0, the conservative worst
-- case, which this function will turn into a maximum-climb floor target).
function GroundProtection:computeFloorTarget(height, distance)
    if not isFiniteNumber(height) or not isFiniteNumber(distance) then
        self.lastAllowedRate = nil
        self.lastFloorTarget = nil
        return -math.huge
    end

    local rate = GroundProtection.allowedDescentRate(distance)
    self.lastAllowedRate = rate

    -- At or above the table's unrestricted rate, AltitudeHold's own descent
    -- clamp already provides this behavior. Reconstructing a target for it
    -- algebraically would be both unnecessary and numerically fragile right
    -- at the clamp boundary, so skip entirely instead.
    if rate <= -AltitudeHold.MAX_DESCENT_RATE then
        self.lastFloorTarget = nil
        return -math.huge
    end

    local floorTarget = GroundProtection.targetForRate(height, rate)
    self.lastFloorTarget = floorTarget
    return floorTarget
end
