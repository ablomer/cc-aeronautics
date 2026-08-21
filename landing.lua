require("util")
require("autopilot")
require("protection")

-- LandingSequence is driven entirely by the burner lever: lever position 0
-- means "land here, straight down," any other position means "fly normally
-- at the mapped altitude" (see main.lua's leverToTargetAltitude / lever
-- position 0 handling). There is no separate arm/cancel button and no
-- independent descent-rate ramp -- moving the lever off 0 is itself the
-- abort, exactly like the steering wheel already reclaims steering from
-- BearingHold by being turned.
--
-- Like protection.lua, this only ever proposes an altitude TARGET, combined
-- with GroundProtection's floor via math.max() in main.lua. It has no
-- separate control law and cannot command burners or propellers directly,
-- and it can never request a descent faster than the floor allows.
--
-- Landing requests an effectively unlimited descent (MIN_ALTITUDE, i.e.
-- 0, far below any real terrain) rather than owning its own ramp rate.
-- GroundProtection.CLEARANCE_TABLE already describes a full flare curve
-- from cruise descent down to rest at the ground; reusing it here means
-- there is exactly one place that shapes the approach, and landing always
-- matches whatever the protection envelope was most recently tuned to.
--
-- DESCENT_MARGIN exists because CLEARANCE_TABLE's rates are a genuine
-- safety limit, not a target -- protection.lua's own module doc explains
-- that "any faster than this is unsafe." If landing rode the safety limit
-- exactly, the floor would be saturated during every routine landing and
-- have no headroom left to actually catch anything unexpected (sensor lag,
-- gust, uneven terrain). Landing instead requests DESCENT_MARGIN of the
-- envelope's allowed rate, so the floor is normally slack and the pilot
-- can watch it engage only if the ship falls behind schedule.
LandingSequence = {}

LandingSequence.DESCENT_MARGIN = 0.6  -- fraction of GroundProtection's allowed rate landing will request

-- Propulsion must be near-zero before descent begins (see design note
-- above on why the flight computer can't actively brake -- propellers are
-- 0-15 only, no reverse. This isn't a request threshold, it's "coasting
-- has had enough time to matter").
LandingSequence.SLOWING_MAX_SPEED = 0.5  -- m/s

LandingSequence.LANDED_DISTANCE      = 1.0  -- metres
LandingSequence.LANDED_SPEED         = 0.3  -- m/s (absolute)
LandingSequence.LANDED_TICKS_REQUIRED = 5   -- ~1s at the 0.2s control period, matching the
                                             -- consecutive-tick pattern already used for sensor hysteresis

local function isFiniteNumber(v)
    return type(v) == "number" and v == v and v ~= math.huge and v ~= -math.huge
end

function LandingSequence:new()
    local t = setmetatable({}, { __index = LandingSequence })
    t.state = "IDLE"  -- "IDLE" | "SLOWING" | "DESCENDING" | "LANDED"
    t.landedTicks = 0
    return t
end

-- Called by main.lua whenever the lever's requested state changes (lever at
-- position 0 vs. not). leverAtZero=true arms/continues landing;
-- leverAtZero=false aborts back to IDLE immediately from any state --
-- moving the lever is an unconditional override, matching how the wheel
-- already reclaims steering.
function LandingSequence:setLeverState(leverAtZero)
    if leverAtZero then
        if self.state == "IDLE" then
            self.state = "SLOWING"
            self.landedTicks = 0
        end
    else
        self.state = "IDLE"
        self.landedTicks = 0
    end
end

-- Whether main.lua should currently force velocity target and throttle
-- lever to 0 and ignore steering input. True for SLOWING and DESCENDING;
-- false for IDLE (nothing to suppress) and LANDED (nothing left to
-- suppress at rest, and this avoids fighting a manual restart -- e.g. the
-- throttle lever alone -- without requiring the pilot to also move the
-- altitude lever off 0 first).
--
-- Deliberately a plain function of self.state, checked by main.lua BEFORE
-- calling read() below (main.lua needs this ahead of the propeller mixers,
-- which run earlier in the control tick than the altitude/sensor reads
-- read() depends on). Because suppression is identical across SLOWING and
-- DESCENDING, and read() can only ever advance the phase forward (never
-- backward into a suppressed state from LANDED), checking state here
-- slightly before vs. after read() runs this same tick cannot disagree
-- about whether to suppress -- only about the exact instant a forward
-- transition takes effect, which is at most one control tick (0.2s) later
-- and inconsequential for a state that was already suppressing propulsion
-- either way.
function LandingSequence:isSuppressingPropulsion()
    return self.state == "SLOWING" or self.state == "DESCENDING"
end

-- Call once per tick while state ~= "IDLE" (main.lua skips this entirely
-- otherwise, same pattern as bearingHold:read() only being consulted while
-- nav steering is active).
--
-- Returns the altitude target to use this tick, or nil if landing has no
-- opinion right now (SLOWING: propulsion hasn't died down enough to start
-- descending yet, so the pilot's last real altitude simply holds via
-- AltitudeHold's own deadband; or DESCENDING with untrustworthy sensor
-- data this tick). Callers should fall back to AltitudeHold's own current
-- target when nil, the same as any tick with nothing new to offer.
--
-- No explicit sensor-fault abort here (unlike the earlier button-driven
-- version of this module): a nil target here already freezes AltitudeHold
-- at its last real target, and OpticalSensorBank's own fail-closed design
-- (a total sensor fault reports distance=0, the worst case, not an invalid
-- number) means allowedDescentRate(0) == 0 flows straight through the
-- normal DESCENDING math below to "hold, don't descend further" without
-- needing a special case. This matches AltitudeHold's own fault
-- philosophy (freeze the last output) rather than introducing a second,
-- different one.
--
-- distance/verticalSpeed/horizontalSpeed are this tick's live readings,
-- same re-read-don't-cache pattern as protection.lua and
-- liftpropeller.lua use, for the same reason: this needs to react within
-- the tick data changes, not one tick behind.
function LandingSequence:read(currentHeight, verticalSpeed, horizontalSpeed, distance)
    if self.state == "SLOWING" then
        if isFiniteNumber(horizontalSpeed) and horizontalSpeed <= LandingSequence.SLOWING_MAX_SPEED then
            self.state = "DESCENDING"
        end
        return nil
    end

    if self.state == "DESCENDING" then
        if isFiniteNumber(distance) and distance <= LandingSequence.LANDED_DISTANCE
        and isFiniteNumber(verticalSpeed) and math.abs(verticalSpeed) <= LandingSequence.LANDED_SPEED then
            self.landedTicks = self.landedTicks + 1
            if self.landedTicks >= LandingSequence.LANDED_TICKS_REQUIRED then
                self.state = "LANDED"
                return currentHeight
            end
        else
            self.landedTicks = 0
        end

        if not isFiniteNumber(currentHeight) or not isFiniteNumber(distance) then
            return nil
        end

        -- Request DESCENT_MARGIN of whatever GroundProtection currently
        -- considers the max SAFE rate at this clearance -- not the safety
        -- limit itself (see module doc above for why) and not an
        -- independent ramp. This keeps landing and the crash-protection
        -- floor reading from the exact same envelope, just at different
        -- fractions of it, so they can never disagree about shape, only
        -- about margin. CLEARANCE_TABLE's rates are all <= 0 (see
        -- protection.lua's own fix note), so this can never request a
        -- climb -- only ever descend, decelerating as distance shrinks.
        local safeRate = GroundProtection.allowedDescentRate(distance)
        local requestedRate = safeRate * LandingSequence.DESCENT_MARGIN
        return GroundProtection.targetForRate(currentHeight, requestedRate)
    end

    -- state == "LANDED": hold position.
    return currentHeight
end
