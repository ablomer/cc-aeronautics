require("flight")
require("controls")
require("autopilot")
require("display")
require("sensors")
require("protection")
require("liftpropeller")
require("landing")
require("util")

local propeller1 = Propeller:new("analog_transmission_8")
local propeller2 = Propeller:new("analog_transmission_9")
local throttleLever = peripheral.wrap("throttle_lever_7")
local velocitySensor = peripheral.wrap("velocity_sensor_3")
local steeringWheel = peripheral.wrap("steering_wheel_3")
local navigationTable = peripheral.wrap("navigation_table_1")
local burnerLever = peripheral.wrap("throttle_lever_8")
local altitudeSensor = peripheral.wrap("altitude_sensor_1")
local burners = {
    peripheral.wrap("hot_air_burner_2")
}
local opticalSensors = OpticalSensorBank:new({ "optical_sensor_0" })
local groundProtection = GroundProtection:new()
local liftPropellerHold = LiftPropellerHold:new("analog_transmission_10")
local landing = LandingSequence:new()

-- Same validity check used locally in protection.lua/landing.lua (each
-- keeps its own copy rather than sharing one, consistent with how this
-- codebase already duplicates small local helpers like isValidHeight
-- across files instead of introducing a shared micro-utility for them).
local function isFiniteNumber(v)
    return type(v) == "number" and v == v and v ~= math.huge and v ~= -math.huge
end

local MAX_POWER = 15
local STEERING_OFFSET = MAX_POWER / 2  -- max differential at full steering lock
local cachedSpeed = 0  -- computed once per tick to avoid double-calling velocityHold:read()
local lastToggleState = redstone.getInput("top")
local lastNavSteering = false

local leverOutput = OnChangeOutput:new(function(v)
    throttleLever.setSignal(v)
end, 1, true)

local velocityHold = VelocityHold:new(
    velocitySensor,
    function(v) leverOutput:set(v) end,
    0, MAX_POWER
)

local burnerBank = BurnerBank:new(burners)
local altitudeHold = AltitudeHold:new(altitudeSensor)
local lastBurnerLever = nil  -- forces target recompute + capture on first tick

-- Lever position 0 is reserved for landing (see landing.lua) and no longer
-- maps to an altitude at all. Positions 1-15 map linearly to
-- [LEVER_MIN_ALTITUDE, AltitudeHold.MAX_ALTITUDE]. LEVER_MIN_ALTITUDE is
-- deliberately its own constant, separate from AltitudeHold.MIN_ALTITUDE
-- (the controller's own safety clamp, now 0): this is only the bottom of
-- the pilot-selectable altitude range, chosen to sit above all real
-- terrain (confirmed: no land below y=65) so the lowest non-landing lever
-- position still corresponds to a safe cruise altitude everywhere.
-- Declared here, ahead of pilotAltitudeTarget and leverToTargetAltitude
-- below, since both depend on it.
local LEVER_MIN_ALTITUDE = 65

-- The lever's requested target, before ground-protection correction.
-- Initialized to a real value (not nil) rather than left unset: if the
-- ship starts with the lever already at position 0 (landing), it would
-- otherwise never get set at all -- landing supplies the effective target
-- in that case, but pilotAltitudeTarget must still be a real number since
-- it's always one operand of math.max() below regardless of lever state.
local pilotAltitudeTarget = LEVER_MIN_ALTITUDE

-- FIXED: leverToTargetAltitude used to map position 1 to a fixed 65m
-- absolute altitude always. That's correct for taking off over the
-- world's lowest terrain, but landing (landing.lua) can rest the ship at
-- ANY real terrain height -- e.g. a hilltop well above 65m. Pushing the
-- lever to 1 there requested 65m, which is BELOW where the ship already
-- was; GroundProtection's floor then held the ship in place (correctly
-- refusing to descend into the hill), which looked from the pilot's seat
-- like "1 does nothing."
--
-- takeoffBaseAltitude rebases the lever's range to the ship's actual
-- height at the moment it leaves position 0 (see the "leaving landing"
-- check in controlUpdate below), so position 1 always means "climb a bit
-- from here" instead of "go to a fixed world altitude." It defaults to
-- LEVER_MIN_ALTITUDE so a session that never touches position 0 behaves
-- exactly as before (position 1 -> 65m over low terrain).
local takeoffBaseAltitude = LEVER_MIN_ALTITUDE

-- Minimum climb requested by moving the lever from 0 to 1, so that leaving
-- the ground always requests going UP from wherever the ship is resting,
-- not just "current height" (which would request zero climb and rely on
-- deadband alone to eventually separate from the ground). Unverified
-- placeholder, tune in-game like the other constants in this system.
local TAKEOFF_CLIMB_STEP = 10

local display = FlightDisplay:new()
local bearingHold = BearingHold:new(navigationTable)

local NAV_DISENGAGE_RANGE = 20  -- metres; hand steering back to wheel within this distance

local function activeSteering()
    -- No lateral movement while landing is actively slowing/descending
    -- (see landing.lua): the pilot asked to go straight down, and the
    -- ship has no way to actively correct drift anyway (propellers are
    -- 0-15, no reverse), so the only thing steering CAN safely do here is
    -- stop contributing anything.
    if landing:isSuppressingPropulsion() then
        return 0
    end
    if navigationTable.hasTarget()
    and navigationTable.getDistanceToTarget() > NAV_DISENGAGE_RANGE then
        return bearingHold:read()
    else
        return (steeringWheel.getAngle() or 0) / 180
    end
end

local function activeSpeed()
    return cachedSpeed
end

-- Position 0 is handled separately in controlUpdate, before this function
-- is ever consulted, using LEVER_MIN_ALTITUDE declared above.
--
-- Uses takeoffBaseAltitude (rebased on leaving position 0, see
-- controlUpdate) rather than LEVER_MIN_ALTITUDE directly, so position 1
-- always requests climbing from wherever the ship actually is. math.max
-- against LEVER_MIN_ALTITUDE guards against ever using a base below the
-- world's known-lowest terrain, in case of a bad height reading during
-- the rebase capture.
local function leverToTargetAltitude(leverPosition)
    local base = math.max(LEVER_MIN_ALTITUDE, takeoffBaseAltitude)
    local span = AltitudeHold.MAX_ALTITUDE - base
    -- leverPosition ranges 1-15 here (0 is handled before this is called);
    -- shift so position 1 maps to base and 15 maps to MAX_ALTITUDE.
    return base + ((leverPosition - 1) / (MAX_POWER - 1)) * span
end

-- Right propeller: base speed + steering offset
local rightMixer = MixerChannel:new(
    function(v) propeller1:setPower(v) end,
    0, MAX_POWER,
    { read = activeSpeed,   weight = 1.0 },
    { read = activeSteering, weight =  STEERING_OFFSET }
)

-- Left propeller: base speed - steering offset (negative weight inverts the differential)
local leftMixer = MixerChannel:new(
    function(v) propeller2:setPower(v) end,
    0, MAX_POWER,
    { read = activeSpeed,   weight = 1.0 },
    { read = activeSteering, weight = -STEERING_OFFSET }
)

local function controlUpdate()
    local toggleState = redstone.getInput("top")

    -- Read the burner lever and update landing's lever-driven state FIRST,
    -- before velocity is computed and the propeller mixers run below --
    -- activeSteering() (called from inside the mixers) and the suppression
    -- check just below both need landing's state for THIS tick, not last
    -- tick's. See landing.lua: position 0 arms/continues landing, any
    -- other position is an unconditional abort back to IDLE, exactly like
    -- turning the wheel already reclaims steering from BearingHold.
    local burnerLeverState = burnerLever.getState()
    local wasSuppressing = landing:isSuppressingPropulsion()
    landing:setLeverState(burnerLeverState == 0)
    local suppressPropulsion = landing:isSuppressingPropulsion()

    -- On the transition INTO suppression, zero velocity hold's target so
    -- that if suppression later ends in hold mode, the ship resumes at
    -- rest instead of leaping back toward a stale nonzero target from
    -- before landing was requested. Same "zero the target on a major
    -- transition" pattern already used below when nav steering disengages.
    if suppressPropulsion and not wasSuppressing then
        velocityHold:setTarget(0)
    end

    -- Detect transition into hold mode and capture current velocity as target
    if toggleState and not lastToggleState then
        leverOutput:invalidate()
        velocityHold:captureTarget(throttleLever.getState())
    end
    lastToggleState = toggleState

    -- Compute base speed once so velocityHold:read() is only called once
    -- per tick. While landing is actively slowing/descending, force zero
    -- regardless of hold/manual mode: the ship can only coast to a stop
    -- here, never actively brake (propellers are 0-15, no reverse -- see
    -- landing.lua's design note), so the only thing left to do is stop
    -- adding thrust. leverOutput:set(0) forces the physical throttle lever
    -- down too, since in manual mode cachedSpeed is read directly from it
    -- every tick further below and would otherwise ignore this entirely.
    if suppressPropulsion then
        cachedSpeed = 0
        leverOutput:set(0)
    elseif toggleState then
        cachedSpeed = velocityHold:read()
    else
        cachedSpeed = throttleLever.getState()
    end

    rightMixer:update()
    leftMixer:update()

    local hasTarget = navigationTable.hasTarget()
    local navSteering = hasTarget and navigationTable.getDistanceToTarget() > NAV_DISENGAGE_RANGE

    -- When nav steering engages, seed the integral from current bearing
    if navSteering and not lastNavSteering then
        bearingHold:captureState()
    end

    -- When nav steering disengages while velocity hold is active, set target to 0
    if lastNavSteering and not navSteering and toggleState then
        velocityHold:setTarget(0)
    end
    lastNavSteering = navSteering

    -- Altitude hold: the burner lever always drives a target altitude (there
    -- is no separate manual/hold switch for this axis, unlike the throttle).
    -- burnerLeverState was already read above (needed earlier for landing's
    -- lever-driven state); not re-read here.
    local isFirstTick = lastBurnerLever == nil

    -- Bumpless startup: seed the integral from the burners' current
    -- commanded amount only on the very first tick, so the controller
    -- doesn't start from zero and cause a jump.
    if isFirstTick then
        altitudeHold:captureTarget(burnerBank.lastAmount)
    end

    -- Retarget whenever the lever moves off position 0 (including the
    -- first tick, so the initial target reflects the lever's starting
    -- position), OR whenever it moves among positions 1-15. Position 0
    -- itself does NOT set pilotAltitudeTarget -- landing (already updated
    -- above) supplies its own target instead; see the effective-target
    -- combination below.
    if burnerLeverState ~= 0 and (isFirstTick or burnerLeverState ~= lastBurnerLever) then
        -- Rebase takeoffBaseAltitude exactly once, on the tick the lever
        -- leaves position 0 (lastBurnerLever == 0), so position 1 always
        -- means "climb TAKEOFF_CLIMB_STEP metres from wherever the ship is
        -- resting" instead of a fixed world altitude (see the FIXED note
        -- above takeoffBaseAltitude's declaration). Uses
        -- altitudeHold.lastHeight -- the most recent CONFIRMED valid
        -- reading -- rather than a fresh altitudeSensor.getHeight() call
        -- here, so a sensor glitch on this exact tick can't rebase takeoff
        -- onto a garbage height. Guarded by isFiniteNumber-equivalent nil
        -- check since lastHeight is nil before the very first successful
        -- AltitudeHold:read().
        if lastBurnerLever == 0 and altitudeHold.lastHeight ~= nil then
            takeoffBaseAltitude = altitudeHold.lastHeight + TAKEOFF_CLIMB_STEP
        end
        pilotAltitudeTarget = leverToTargetAltitude(burnerLeverState)
    end
    lastBurnerLever = burnerLeverState

    opticalSensors:read()

    -- Ground protection: if the closest optical reading implies the pilot's
    -- requested target would let AltitudeHold descend faster than is safe
    -- for the remaining clearance, raise the effective target instead of
    -- reaching for a second, competing control path. AltitudeHold's own
    -- cascade (outer loop -> inner PI -> slew) is reused unchanged; there is
    -- still exactly one target and one call to burnerBank:setAmount.
    --
    -- currentHeight is read directly here (rather than reusing
    -- altitudeHold.lastHeight, which is one tick stale) so the correction
    -- reacts to this tick's height, not last tick's. AltitudeHold:read()
    -- below will read the sensor again itself; this mirrors the existing
    -- pattern of the display separately re-reading velocitySensor for
    -- telemetry rather than caching VelocityHold's internal reading.
    local currentHeight = altitudeSensor.getHeight()
    local currentVerticalSpeed = altitudeSensor.getVerticalSpeed()

    -- FIXED: on entering suppression (landing engaging), immediately stop
    -- demanding the pilot's old altitude target and reseed AltitudeHold's
    -- integral to match. Without this, if that old target was above the
    -- ship's real physical ceiling (burner authority maxes out below the
    -- requested height -- confirmed in-game: ~150m at full 500 burner),
    -- the outer loop had been demanding max climb rate indefinitely (since
    -- height never approaches an unreachable target) and the integral had
    -- wound up toward its limit in response. That integral only unwinds by
    -- rateError * dt per tick once actually descending, which meant a long
    -- real-world delay -- reported in-game as "burner takes a while to
    -- drop below 75 and start descending" -- before descent could begin at
    -- all. Lowering RATE_I_LIMIT (autopilot.lua) bounds how bad this can
    -- get in general, but reseeding here removes the delay entirely for
    -- the landing case specifically, which is the one guaranteed to
    -- trigger it (any lever-1 cruise target can be set arbitrarily high).
    --
    -- Reseeding to currentHeight (not e.g. 0) matches the SLOWING phase's
    -- own intent: hold roughly in place while horizontal speed bleeds off,
    -- not climb OR dive. altitudeHold:captureTarget() backs the integral
    -- out from lastAmount the same bumpless way main.lua's own first-tick
    -- startup already does, so there's no discontinuity in commanded
    -- burner amount at the moment of reseed, only in what the outer loop
    -- is asking for going forward.
    if suppressPropulsion and not wasSuppressing and isFiniteNumber(currentHeight) then
        pilotAltitudeTarget = currentHeight
        altitudeHold:setTarget(currentHeight)
        altitudeHold:captureTarget(altitudeHold.lastAmount)
    end

    -- Horizontal speed for landing's arm/abort checks. velocitySensor
    -- measures the ship's forward velocity (the same reading VelocityHold
    -- already holds/targets for propulsion) -- NOT a true omnidirectional
    -- horizontal speed, so lateral drift the velocity sensor can't see
    -- would not be caught here. Good enough for an initial landing gate;
    -- revisit if the ship turns out to drift sideways during descent.
    local horizontalSpeed = velocitySensor.getVelocity()

    -- Landing: while lever position 0 has landing active (state ~= IDLE),
    -- read() proposes a descent target that eases toward the ground using
    -- GroundProtection's own envelope, at a margin below its safety limit
    -- (see landing.lua). Combined with the ground-protection floor below
    -- via math.max(), so it can request descent below the pilot's lever
    -- position but can never request descent below the always-on
    -- crash-protection floor. A nil return (landing has no opinion this
    -- tick -- e.g. still SLOWING, or IDLE) falls back to the lever's own
    -- target unchanged.
    local landingTarget = nil
    if landing.state ~= "IDLE" then
        landingTarget = landing:read(currentHeight, currentVerticalSpeed, horizontalSpeed, opticalSensors.lastMinDistance)
    end
    local baseAltitudeTarget = landingTarget or pilotAltitudeTarget

    local floorTarget = groundProtection:computeFloorTarget(currentHeight, opticalSensors.lastMinDistance)
    local effectiveAltitudeTarget = math.max(baseAltitudeTarget, floorTarget)
    local groundProtectionActive = floorTarget > baseAltitudeTarget
    altitudeHold:setTarget(effectiveAltitudeTarget)

    local burnerAmount = altitudeHold:read()
    burnerBank:setAmount(burnerAmount)

    -- Lift propellers: fast-acting braking assist, always live (this is the
    -- "always-on crash prevention" layer -- there is no arm/disarm switch
    -- for it). Reads this tick's clearance/vertical-speed directly rather
    -- than reusing altitudeHold.lastVerticalSpeed (one tick stale), for the
    -- same reason the ground-protection target correction above re-reads
    -- the sensor itself instead of trusting stale state.
    local liftPropellerPower = liftPropellerHold:read(opticalSensors.lastMinDistance, currentVerticalSpeed)

    display:update({
        velocity       = velocitySensor.getVelocity(),
        throttle       = throttleLever.getState(),
        holdMode       = toggleState,
        targetVelocity = velocityHold.target,
        steeringAngle  = steeringWheel.getAngle() or 0,
        navActive      = hasTarget,
        navBearing     = hasTarget and navigationTable.getBearing() or nil,
        navHeading     = navigationTable.getHeading(),
        navOutput      = bearingHold.lastOutput,
        navDistance    = hasTarget and navigationTable.getDistanceToTarget() or nil,
        navSteering    = navSteering,
        propeller1Power = propeller1.lastPower,
        propeller2Power = propeller2.lastPower,
        altitude       = altitudeHold.lastHeight,
        targetAltitude = altitudeHold.target,
        verticalSpeed  = altitudeHold.lastVerticalSpeed,
        burnerAmount   = burnerBank.lastAmount,
        altitudeFault  = altitudeHold.fault,
        groundDistance = opticalSensors.lastMinDistance,
        groundSensor   = opticalSensors.lastMinSensor,
        groundFault    = opticalSensors.fault,
        groundProtectionActive = groundProtectionActive,
        liftPropellerPower = liftPropellerPower,
        landingState = landing.state,
        horizontalSpeed = horizontalSpeed,
    })
end

-- Survivability wrapper: an uncaught error inside controlUpdate would
-- otherwise crash this program entirely, leaving actuators latched at
-- whatever was last commanded, with no way to recover short of physically
-- re-running the program. pcall keeps the event loop alive across a
-- faulted tick and forces both vertical actuators to a defined safe state
-- immediately:
--   - burners to their minimum, since we can't tell if the fault means
--     "AltitudeHold's own state is unreliable now" and unattended heat is
--     a hazard on its own.
--   - lift propellers to MAX_POWER. This differs from the burner's
--     fail-to-minimum on purpose: with the control loop itself faulted, we
--     have no verified vertical-speed reading and, given the ship's
--     momentum, cannot assume it isn't already falling. Full propeller
--     power is the conservative choice here the same way opticalSensors
--     and groundProtection already fail toward "assume the worst" rather
--     than "assume clear".
--
-- We don't know which peripherals controlUpdate already wrote before the
-- error occurred, so we don't try to guess at a consistent partial state --
-- we just force the two actuators we know how to force safely.
local controlFault = false
local lastControlError = nil

local function safeControlUpdate()
    local ok, err = pcall(controlUpdate)
    if not ok then
        controlFault = true
        lastControlError = tostring(err)
        pcall(function() burnerBank:setAmount(BURNER_AMOUNT_RANGE.min) end)
        pcall(function() liftPropellerHold.propeller:setPower(LiftPropellerHold.MAX_POWER) end)
        pcall(function()
            display:update({
                velocity = 0, throttle = 0, holdMode = false,
                steeringAngle = 0, navActive = false,
                propeller1Power = 0, propeller2Power = 0,
                altitude = altitudeHold.lastHeight, targetAltitude = altitudeHold.target,
                verticalSpeed = altitudeHold.lastVerticalSpeed,
                burnerAmount = burnerBank.lastAmount, altitudeFault = true,
                groundDistance = opticalSensors.lastMinDistance,
                groundSensor = opticalSensors.lastMinSensor,
                groundFault = opticalSensors.fault,
                liftPropellerPower = liftPropellerHold.propeller.lastPower,
                landingState = landing.state,
                controlFault = true, controlError = lastControlError,
            })
        end)
    else
        controlFault = false
        lastControlError = nil
    end
end

local timer = os.startTimer(0.2)
while true do
    local event, p1, p2, p3 = os.pullEvent()

    if event == "timer" and p1 == timer then
        safeControlUpdate()
        timer = os.startTimer(0.2)

    elseif event == "mouse_click" then
        local action = display:hitTest(p2, p3)
        if action == "inc" then
            local rounded = math.floor(velocityHold.target * 10 + 0.5) / 10
            velocityHold:nudgeTarget(rounded - velocityHold.target + 0.1)
        elseif action == "dec" then
            local rounded = math.floor(velocityHold.target * 10 + 0.5) / 10
            velocityHold:nudgeTarget(rounded - velocityHold.target - 0.1)
        elseif action == "zero" then
            velocityHold:setTarget(0)
        elseif action == "two" then
            velocityHold:setTarget(2)
        end
    end
end
