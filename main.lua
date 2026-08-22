require("flight")
require("controls")
require("autopilot")
require("display")
require("util")
require("config")


-- ------------------------
-- LNAV
-- ------------------------
local propeller1 = Propeller:new(PERIPHERALS.LNAV.rightPropellerTransmission)
local propeller2 = Propeller:new(PERIPHERALS.LNAV.leftPropellerTransmission)
local throttleLever = peripheral.wrap(PERIPHERALS.LNAV.throttleLever)
local velocitySensor = peripheral.wrap(PERIPHERALS.LNAV.velocitySensor)

local steeringWheel = peripheral.wrap(PERIPHERALS.LNAV.steeringWheel)
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
local verticalPropellers = Propeller:new(PERIPHERALS.VNAV.verticalPropellerTransmission)
local opticalSensors = {}
for _, sensorId in ipairs(PERIPHERALS.VNAV.opticalSensors) do
    table.insert(opticalSensors, peripheral.wrap(sensorId))
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
local verticalSpeedHold = VerticalSpeedHold:new(altitudeSensor)
local altitudeHold = AltitudeHold:new()
local lastBurnerLever = nil  -- forces target recompute + capture on first tick
local landedLatched = false  -- stays true after touchdown until the lever leaves 0

local display = FlightDisplay:new()
local bearingHold = BearingHold:new(navigationTable)

local NAV_DISENGAGE_RANGE = 20  -- metres; hand steering back to wheel within this distance

local function activeSteering()
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

-- Maps the 1-15 burner lever position to a target altitude within the
-- operational range. Lever 0 is the landing detent and does not use this
-- mapping. MAX_ALTITUDE (315) is used instead of the sensor's true ceiling
-- (320) to leave braking margin; see VerticalSpeedHold.HARD_CEILING.
local function leverToTargetAltitude(leverPosition)
    local span = AltitudeHold.MAX_ALTITUDE - AltitudeHold.MIN_ALTITUDE
    return AltitudeHold.MIN_ALTITUDE + (leverPosition / MAX_POWER) * span
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

    -- Detect transition into hold mode and capture current velocity as target
    if toggleState and not lastToggleState then
        leverOutput:invalidate()
        velocityHold:captureTarget(throttleLever.getState())
    end
    lastToggleState = toggleState

    -- Compute base speed once so velocityHold:read() is only called once per tick
    if toggleState then
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
    })
end

local timer = os.startTimer(0.1)
while true do
    local event, p1, p2, p3 = os.pullEvent()

    if event == "timer" and p1 == timer then
        controlUpdate()
        timer = os.startTimer(0.1)

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
