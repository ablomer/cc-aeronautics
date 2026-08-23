require("flight")
require("controls")
require("autopilot")
require("display")
require("audio")
require("util")
require("config")


-- ------------------------
-- LNAV
-- ------------------------
local propeller1 = Propeller:new(PERIPHERALS.LNAV.leftPropellerSpeedController, {
    maxRpm = SHIP.LNAV.maxRpm,
    invert = SHIP.LNAV.invertLeft,
})
local propeller2 = Propeller:new(PERIPHERALS.LNAV.rightPropellerSpeedController, {
    maxRpm = SHIP.LNAV.maxRpm,
    invert = SHIP.LNAV.invertRight,
})
local throttleLever = peripheral.wrap(PERIPHERALS.LNAV.throttleLever)
local velocitySensor = peripheral.wrap(PERIPHERALS.LNAV.velocitySensor)

local steeringWheel = SteeringWheel:new(PERIPHERALS.LNAV.steeringWheel, {
    deadzone = SHIP.LNAV.steeringDeadzone,
})
local navigationTable = peripheral.wrap(PERIPHERALS.LNAV.navigationTable)

-- ------------------------
-- VNAV
-- ------------------------
local burnerLever = peripheral.wrap(PERIPHERALS.VNAV.burnerLever)
local altitudeSensor = peripheral.wrap(PERIPHERALS.VNAV.altitudeSensor)
local burners = {}
for _, burnerId in ipairs(PERIPHERALS.VNAV.burners) do
    table.insert(burners, peripheral.wrap(burnerId))
end

-- Vertical propellers all controlled by the same analog transmission
local verticalPropellers = AnalogPropeller:new(PERIPHERALS.VNAV.verticalPropellerTransmission)
local opticalSensors = {}
for _, sensorId in ipairs(PERIPHERALS.VNAV.opticalSensors) do
    table.insert(opticalSensors, peripheral.wrap(sensorId))
end

local MAX_POWER = 15  -- throttle / burner lever notches (still 0-15)
local MAX_RPM = SHIP.LNAV.maxRpm
local MAX_STEER_DIFF = SHIP.LNAV.maxSteerDiff  -- RPM differential at full lock
local cachedSpeed = 0  -- computed once per tick to avoid double-calling velocityHold:read()
local lastNavSteering = false

local velocityHold = VelocityHold:new(velocitySensor, 0, MAX_RPM)

local burnerBank = BurnerBank:new(burners)
local verticalSpeedHold = VerticalSpeedHold:new(altitudeSensor)
local altitudeHold = AltitudeHold:new()
local lastBurnerLever = nil  -- forces target recompute + capture on first tick
local landedLatched = false  -- stays true after touchdown until the lever leaves 0

-- ------------------------
-- ATT
-- ------------------------
local gimbal = peripheral.wrap(PERIPHERALS.ATT.gimbalSensor)
local stabilizer = Servo:new(
    PERIPHERALS.ATT.stabilizerSpeedController,
    PERIPHERALS.ATT.stabilizerBearing,
    SHIP.ATT
)
local pitchHold = PitchHold:new(gimbal)
local pitchSign = SHIP.ATT.invertPitch and -1 or 1

local display = FlightDisplay:new()
local speakers = {}
for _, speakerId in ipairs(PERIPHERALS.AUDIO.speakers) do
    table.insert(speakers, peripheral.wrap(speakerId))
end
local audio = ShipAudio:new(speakers)
local headingHold = HeadingHold:new(navigationTable, gimbal)

local NAV_DISENGAGE_RANGE = 20  -- metres; hand steering back to wheel within this distance

-- NAV writes the heading setpoint from the compass bearing whenever a
-- target is resolved and farther than the arrival disk. No throttle gate:
-- the hull can pivot to face the destination before the lever opens.
local function wantNavSteering()
    return navigationTable.hasTarget()
        and navigationTable.getDistanceToTarget() > NAV_DISENGAGE_RANGE
end

-- Wheel deflection -> commanded heading-slew rate (deg/s). Deadzone is
-- already applied inside getNormalized(); the exponent gives finer
-- control near center.
local function wheelTurnRate()
    local x = steeringWheel:getNormalized()
    if x == 0 then return 0 end
    local sign = x < 0 and -1 or 1
    return sign * (math.abs(x) ^ SHIP.LNAV.turnRateExponent) * SHIP.LNAV.maxTurnRate
end

-- invertSteer is actuator-side only. Do not also flip the heading error
-- or the loop will spin instead of settling.
local function applySteerSign(steer)
    if SHIP.LNAV.invertSteer then
        return -steer
    end
    return steer
end

-- Maps the 1-15 burner lever position to a target altitude within the
-- operational range. Lever 0 is the landing detent and does not use this
-- mapping. MAX_ALTITUDE (315) is used instead of the sensor's true ceiling
-- (320) to leave braking margin; see VerticalSpeedHold.HARD_CEILING.
local function leverToTargetAltitude(leverPosition)
    local span = AltitudeHold.MAX_ALTITUDE - AltitudeHold.MIN_ALTITUDE
    return AltitudeHold.MIN_ALTITUDE + (leverPosition / MAX_POWER) * span
end

-- Maps the 0-15 throttle lever onto 0 .. SHIP.LNAV.maxSpeed. Detent 0 is stop.
local function leverToTargetVelocity(leverPosition)
    return (leverPosition / MAX_POWER) * SHIP.LNAV.maxSpeed
end

local function controlUpdate()
    -- VNAV preamble first: heading hold needs landedLatched, and the
    -- first-tick captures share this latch.
    local burnerLeverState = burnerLever.getState()
    local isFirstTick = lastBurnerLever == nil
    local landing = burnerLeverState == 0

    if isFirstTick then
        verticalSpeedHold:capture(burnerBank.lastAmount)
        headingHold:captureHeading()
    end

    -- Retarget whenever the lever moves in hold (including the first tick
    -- if we start above detent 0). Landing does not use an altitude target.
    if not landing and (isFirstTick or burnerLeverState ~= lastBurnerLever) then
        altitudeHold:setTarget(leverToTargetAltitude(burnerLeverState))
    end
    lastBurnerLever = burnerLeverState

    local agl, hasGround = readWorstAgl(opticalSensors)
    local wasLanded = landedLatched
    if not landing then
        landedLatched = false
    elseif isTouchdown(agl, hasGround) then
        landedLatched = true
    end
    if wasLanded and not landedLatched then
        headingHold:captureHeading()
    end

    -- Heading before surge: the yaw differential has first claim on the
    -- RPM budget, so the velocity loop has to be told the ceiling that
    -- leaves it before it integrates against a limit it cannot reach.
    local hasTarget = navigationTable.hasTarget()
    local navSteering = (not landedLatched) and wantNavSteering()

    -- Freeze the heading setpoint on the current heading before this
    -- tick's HDG read, so we do not track the last NAV target for one
    -- more frame. Throttle is zeroed after the mix, same as before.
    if lastNavSteering and not navSteering then
        headingHold:captureHeading()
    end

    local commandedTurnRate = 0
    local steer
    if landedLatched then
        steer = headingHold:holdOff()
    elseif navSteering then
        local heading = compassHeading(navigationTable.getHeading())
        local bearing = navigationTable.getBearing()
        if isFiniteNumber(heading) and isFiniteNumber(bearing) then
            headingHold:setTarget(norm360(heading + wrap180(bearing)))
        end
        steer = headingHold:read(nil)
    else
        commandedTurnRate = wheelTurnRate()
        steer = headingHold:read(commandedTurnRate)
    end
    steer = applySteerSign(steer)

    -- Throttle lever is always a velocity setpoint. Detent 0 parks the
    -- speed loop; heading hold still yaws the hull in place when airborne.
    local leverState = throttleLever.getState()
    local surgeCeiling = surgeHeadroom(steer, MAX_RPM, MAX_STEER_DIFF)
    if leverState == 0 then
        cachedSpeed = velocityHold:holdOff()
    else
        velocityHold:setTarget(leverToTargetVelocity(leverState))
        velocityHold:setCeiling(surgeCeiling)
        cachedSpeed = velocityHold:read()
    end

    local leftRpm, rightRpm = allocatePropMix(
        cachedSpeed,
        steer,
        MAX_RPM,
        MAX_STEER_DIFF
    )
    propeller1:setSpeed(leftRpm)
    propeller2:setSpeed(rightRpm)

    -- Arrive / NAV drop: zero the physical lever and the speed target
    -- together. Heading was already frozen above.
    if lastNavSteering and not navSteering then
        throttleLever.setSignal(0)
        velocityHold:setTarget(0)
    end
    lastNavSteering = navSteering

    local desiredVS
    local vnavMode
    if landedLatched then
        -- Hull is on the ground: cut heat and props so the VS loop does
        -- not hunt around DVS 0. Stays latched until the lever leaves 0.
        desiredVS = 0
        vnavMode = "landed"
        burnerBank:setAmount(verticalSpeedHold:holdOff())
        verticalPropellers:setPower(0)
    elseif landing then
        desiredVS = landingDesiredVS(agl, hasGround)
        vnavMode = hasGround and "flare" or "land"
        burnerBank:setAmount(verticalSpeedHold:read(desiredVS))
        local propPower = 0
        if not verticalSpeedHold.fault then
            propPower = verticalPropPower(
                verticalSpeedHold.lastRateError,
                verticalSpeedHold.slewLimited,
                hasGround
            )
        end
        verticalPropellers:setPower(propPower)
    else
        local height = altitudeSensor.getHeight()
        local altRate = altitudeHold:desiredRate(height)
        local terrainVS = terrainClimbVS(agl, hasGround)
        -- terrainVS is 0 when nothing has hit (or AGL is at/above clearance).
        -- Do not max() against that 0: it would block descents and falsely
        -- report TERRAIN whenever altitude hold asks to go down.
        desiredVS = altRate
        if terrainVS > 0 then
            desiredVS = math.max(altRate, terrainVS)
        end
        vnavMode = (terrainVS > 0 and terrainVS > altRate) and "terrain" or "hold"
        burnerBank:setAmount(verticalSpeedHold:read(desiredVS))
        local propPower = 0
        if not verticalSpeedHold.fault then
            propPower = verticalPropPower(
                verticalSpeedHold.lastRateError,
                verticalSpeedHold.slewLimited,
                hasGround
            )
        end
        verticalPropellers:setPower(propPower)
    end

    local lnavMode
    if leverState == 0 then
        lnavMode = "stop"
    elseif navSteering then
        lnavMode = "nav"
    else
        lnavMode = "hold"
    end

    -- ATT: hold gimbal pitch at 0 via the stabilizer servo. Park the
    -- loop on the ground so it does not fight the hull sitting still.
    local attMode
    if landedLatched then
        attMode = "off"
        pitchHold:captureState()
        stabilizer:setTarget(0)
    else
        attMode = "level"
        stabilizer:setTarget(pitchHold:read() * pitchSign)
    end
    stabilizer:update()

    local snapshot = {
        velocity       = velocitySensor.getVelocity(),
        throttle       = leverState,
        targetVelocity = velocityHold.target,
        lnavMode       = lnavMode,
        steeringAngle  = steeringWheel:getAngle(),
        heading        = headingHold.lastHeading,
        targetHeading  = headingHold.target,
        commandedTurnRate = commandedTurnRate,
        lnavFault      = headingHold.fault,
        navActive      = hasTarget,
        navBearing     = hasTarget and wrap180(navigationTable.getBearing()) or nil,
        navHeading     = headingHold.lastHeading,
        navOutput      = headingHold.lastOutput,
        navDistance    = hasTarget and navigationTable.getDistanceToTarget() or nil,
        navSteering    = navSteering,
        propeller1Rpm  = propeller1.lastSpeed,
        propeller2Rpm  = propeller2.lastSpeed,
        altitude       = verticalSpeedHold.lastHeight,
        targetAltitude = altitudeHold.target,
        landing        = landing,
        vnavMode       = vnavMode,
        verticalSpeed  = verticalSpeedHold.lastVerticalSpeed,
        desiredVS      = desiredVS,
        agl            = hasGround and agl or nil,
        verticalPropPower = verticalPropellers.lastPower,
        burnerAmount   = burnerBank.lastAmount,
        altitudeFault  = verticalSpeedHold.fault,
        pitch          = pitchHold.lastPitch,
        pitchRate      = pitchHold.lastPitchRate,
        stabAngle      = stabilizer.lastAngle,
        stabTarget     = stabilizer.target,
        stabRpm        = stabilizer.lastSpeed or 0,
        attMode        = attMode,
        attFault       = pitchHold.fault or stabilizer.fault,
    }
    display:update(snapshot)
    audio:update(snapshot)
end


local timer = os.startTimer(0.1)
while true do
    local event, p1 = os.pullEvent()

    if event == "timer" and p1 == timer then
        controlUpdate()
        timer = os.startTimer(0.1)
    end
end
