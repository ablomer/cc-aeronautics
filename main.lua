require("flight")
require("controls")
require("autopilot")
require("display")
require("util")
require("config")


-- ------------------------
-- LNAV
-- ------------------------
local propeller1 = Propeller:new(PERIPHERALS.LNAV.rightPropellerTransmission.id)
local propeller2 = Propeller:new(PERIPHERALS.LNAV.leftPropellerTransmission.id)
local throttleLever = peripheral.wrap(PERIPHERALS.LNAV.throttleLever.id)
local velocitySensor = peripheral.wrap(PERIPHERALS.LNAV.velocitySensor.id)

local steeringWheel = peripheral.wrap(PERIPHERALS.LNAV.steeringWheel.id)
local navigationTable = peripheral.wrap(PERIPHERALS.LNAV.navigationTable.id)

-- ------------------------
-- VNAV
-- ------------------------
local burnerLever = peripheral.wrap(PERIPHERALS.VNAV.burnerLever.id)
local altitudeSensor = peripheral.wrap(PERIPHERALS.VNAV.altitudeSensor.id)
local burners = {}
for _, burner in ipairs(PERIPHERALS.VNAV.burners) do
    table.insert(burners, peripheral.wrap(burner.id))
end

-- Vertical propellers all controlled by the same analog transmission
local verticalPropellers = Propeller:new(PERIPHERALS.VNAV.verticalPropellerTransmission.id)
local opticalSensors = {}
for _, sensor in ipairs(PERIPHERALS.VNAV.opticalSensors) do
    table.insert(opticalSensors, peripheral.wrap(sensor.id))
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

-- Maps the 0-15 burner lever position to a target altitude within the
-- operational range. MAX_ALTITUDE (315) is used instead of the sensor's
-- true ceiling (320) to leave braking margin; see AltitudeHold.HARD_CEILING.
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

    -- Altitude hold: the burner lever always drives a target altitude (there
    -- is no separate manual/hold switch for this axis, unlike the throttle).
    local burnerLeverState = burnerLever.getState()
    local isFirstTick = lastBurnerLever == nil

    -- Bumpless startup: seed the integral from the burners' current
    -- commanded amount only on the very first tick, so the controller
    -- doesn't start from zero and cause a jump.
    if isFirstTick then
        altitudeHold:captureTarget(burnerBank.lastAmount)
    end

    -- Retarget whenever the lever moves (including the first tick, so the
    -- initial target reflects the lever's starting position).
    if isFirstTick or burnerLeverState ~= lastBurnerLever then
        altitudeHold:setTarget(leverToTargetAltitude(burnerLeverState))
    end
    lastBurnerLever = burnerLeverState

    local burnerAmount = altitudeHold:read()
    burnerBank:setAmount(burnerAmount)

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
    })
end

local timer = os.startTimer(0.2)
while true do
    local event, p1, p2, p3 = os.pullEvent()

    if event == "timer" and p1 == timer then
        controlUpdate()
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
