require("util")
require("flight")
require("protection")

-- LiftPropellerHold drives the bottom-mounted lift-propeller bank: a fast,
-- proportional-only braking assist that complements AltitudeHold's much
-- slower burner control (see protection.lua for the burner-side half of
-- ground protection).
--
-- Confirmed hardware: all lift propellers are wired to a single controller
-- and always spin at one shared speed (same setup as the rear propellers,
-- short spin-up latency), so this wraps a single Propeller (flight.lua)
-- rather than fanning a command out to a bank of independently-addressed
-- devices like BurnerBank does.
--
-- Deliberately has NO integral, unlike AltitudeHold's burner PI: an
-- integral here would let this actuator wind up independently of the
-- burner controller and create a second, competing correction on top of
-- protection.lua's target-based one. Confirmed short spin-up latency means
-- a live proportional term should respond fast enough on its own; if
-- in-game testing shows lag or overshoot, a slew limit or integral term can
-- be added later.
--
-- Shares GroundProtection.allowedDescentRate(distance) so the propeller and
-- the burner target correction always agree on what "too fast" means for a
-- given clearance -- there is exactly one safety envelope table
-- (protection.lua's CLEARANCE_TABLE), not two that could drift apart.
LiftPropellerHold = {}

LiftPropellerHold.MAX_POWER = 15

-- Power units commanded per m/s of unsafe descent (i.e. how far below the
-- allowed rate the ship's actual vertical speed is). Unverified placeholder
-- -- tune in-game against the propellers' actual climb authority. At this
-- gain, a 3 m/s deficit already saturates at MAX_POWER.
LiftPropellerHold.GAIN = 5.0

function LiftPropellerHold:new(transmissionName)
    local t = setmetatable({}, { __index = LiftPropellerHold })
    t.propeller = Propeller:new(transmissionName)
    t.lastDeficit = 0
    return t
end

-- distance: closest optical clearance reading (0-15), already normalized by
-- OpticalSensorBank -- a sensor fault reports 0 there (the conservative
-- worst case), which this function turns into near-maximum power via
-- GroundProtection's clearance table. This is the same fail-closed behavior
-- protection.lua's burner-side correction gets from the same input.
--
-- verticalSpeed: current vertical speed. If it isn't a valid number (e.g.
-- the altitude sensor itself is faulted), this commands MAX_POWER outright
-- rather than standing down: with no rate data we cannot confirm the ship
-- isn't falling, and given the ship's momentum, assuming the worst is the
-- safer failure mode here.
--
-- Call once per tick; immediately commands the propeller (no intermediate
-- "read but don't apply" step, since this actuator is meant to react within
-- the same tick clearance/rate data changes).
function LiftPropellerHold:read(distance, verticalSpeed)
    local deficit

    if type(verticalSpeed) ~= "number" or verticalSpeed ~= verticalSpeed then
        deficit = LiftPropellerHold.MAX_POWER / LiftPropellerHold.GAIN
    else
        local safeRate = GroundProtection.allowedDescentRate(distance)
        deficit = safeRate - verticalSpeed  -- positive when descending faster than safe
    end
    self.lastDeficit = deficit

    local power = 0
    if deficit > 0 then
        power = Range:new(0, LiftPropellerHold.MAX_POWER):clamp(deficit * LiftPropellerHold.GAIN)
    end

    self.propeller:setPower(power)
    return power
end
