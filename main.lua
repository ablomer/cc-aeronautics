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
local propeller1 = Propeller:new(PERIPHERALS.LNAV.rightPropellerSpeedController, {
    maxRpm = SHIP.LNAV.maxRpm,
    invert = SHIP.LNAV.invertRight,
})
local propeller2 = Propeller:new(PERIPHERALS.LNAV.leftPropellerSpeedController, {
    maxRpm = SHIP.LNAV.maxRpm,
    invert = SHIP.LNAV.invertLeft,
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
local STEERING_OFFSET = MAX_RPM / 2  -- max differential at full steering lock
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
local bearingHold = BearingHold:new(navigationTable)

local NAV_DISENGAGE_RANGE = 20  -- metres; hand steering back to wheel within this distance

-- Auto steering only while moving. Lever 0 is stop: velocity loop parked,
-- wheel still yaws in place (deadzone keeps props at 0 when centered).
local function wantNavSteering()
    return throttleLever.getState() > 0
        and navigationTable.hasTarget()
        and navigationTable.getDistanceToTarget() > NAV_DISENGAGE_RANGE
end

local function activeSteering()
    local steer
    if wantNavSteering() then
        steer = bearingHold:read()
    else
        steer = steeringWheel:getAngle() / 180
    end
    if SHIP.LNAV.invertSteer then
        steer = -steer
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
    -- Throttle lever is always a velocity setpoint. Detent 0 parks the
    -- speed loop; the wheel can still pivot the hull in place.
    local leverState = throttleLever.getState()
    if leverState == 0 then
        cachedSpeed = velocityHold:holdOff()
    else
        velocityHold:setTarget(leverToTargetVelocity(leverState))
        cachedSpeed = velocityHold:read()
    end
    local leftRpm, rightRpm = allocatePropMix(
        cachedSpeed,
        activeSteering(),
        MAX_RPM,
        STEERING_OFFSET,
        leverState == 0
    )
    propeller2:setSpeed(leftRpm)
    propeller1:setSpeed(rightRpm)

    local hasTarget = navigationTable.hasTarget()
    local navSteering = wantNavSteering()

    -- When nav steering engages, seed the integral from current bearing
    if navSteering and not lastNavSteering then
        bearingHold:captureState()
    end

    -- Arrive: zero the physical lever and the speed target together.
    if lastNavSteering and not navSteering then
        throttleLever.setSignal(0)
        velocityHold:setTarget(0)
    end
    lastNavSteering = navSteering

    -- VNAV: lever 0 is the landing detent (flare on worst-case AGL).
    -- Lever 1-15 is altitude hold, with a terrain climb override from AGL.
    local burnerLeverState = burnerLever.getState()
    local isFirstTick = lastBurnerLever == nil
    local landing = burnerLeverState == 0

    -- Bumpless startup: seed the VS-hold integral from the burners' current
    -- commanded amount only on the very first tick, so the controller
    -- doesn't start from zero and cause a jump.
    if isFirstTick then
        verticalSpeedHold:capture(burnerBank.lastAmount)
    end

    -- Retarget whenever the lever moves in hold (including the first tick
    -- if we start above detent 0). Landing does not use an altitude target.
    if not landing and (isFirstTick or burnerLeverState ~= lastBurnerLever) then
        altitudeHold:setTarget(leverToTargetAltitude(burnerLeverState))
    end
    lastBurnerLever = burnerLeverState

    local agl, hasGround = readWorstAgl(opticalSensors)
    if not landing then
        landedLatched = false
    elseif isTouchdown(agl, hasGround) then
        landedLatched = true
    end

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
        navActive      = hasTarget,
        navBearing     = hasTarget and navigationTable.getBearing() or nil,
        navHeading     = navigationTable.getHeading(),
        navOutput      = bearingHold.lastOutput,
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
