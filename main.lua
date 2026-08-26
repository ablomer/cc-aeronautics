require("util")
require("flight")
require("autopilot")
require("display")
require("audio")
require("config")
require("controls")


-- ------------------------
-- LNAV
-- ------------------------
local leftPropeller = Propeller:new(PERIPHERALS.LNAV.leftPropellerSpeedController, {
    maxRpm = SHIP.LNAV.maxRpm,
    invert = SHIP.LNAV.invertLeft,
})
local rightPropeller = Propeller:new(PERIPHERALS.LNAV.rightPropellerSpeedController, {
    maxRpm = SHIP.LNAV.maxRpm,
    invert = SHIP.LNAV.invertRight,
})
local throttleLever = peripheral.wrap(PERIPHERALS.LNAV.throttleLever)
local velocitySensor = findPeripheral("velocity_sensor")
local steeringWheel = findPeripheral("steering_wheel")
local navigationTable = findPeripheral("navigation_table")
local gimbal = findPeripheral("gimbal_sensor")
local thrustMixer = DifferentialThrustMixer:new(leftPropeller, rightPropeller, SHIP.LNAV)

-- ------------------------
-- VNAV
-- ------------------------
local burnerLever = peripheral.wrap(PERIPHERALS.VNAV.burnerLever)
local altitudeSensor = findPeripheral("altitude_sensor")
local burners = findPeripherals("hot_air_burner")

-- Vertical propellers all controlled by the same rotational speed controller
local verticalPropellers = Propeller:new(PERIPHERALS.VNAV.verticalPropellerSpeedController, {
    maxRpm = SHIP.VNAV.maxRpm,
    invert = SHIP.VNAV.invert,
})
local opticalSensors = findPeripherals("optical_sensor")

local MAX_POWER = 15  -- throttle lever notches (0-15)

local velocityHold = VelocityHold:new(velocitySensor)

local burnerBank = BurnerBank:new(burners)
local altitudeHold = AltitudeHold:new(altitudeSensor)
local volumeCaptured = false
local landedLatched = false  -- stays true after touchdown until the lever leaves 0

-- ------------------------
-- ATT
-- ------------------------
local stabilizer = Servo:new(
    PERIPHERALS.ATT.stabilizerSpeedController,
    PERIPHERALS.ATT.stabilizerBearing,
    SHIP.ATT
)
-- Pitch hold needs the stabilizer RSC and bearing. Either missing
-- disables ATT rather than faulting. The gimbal is already required.
local attAvailable = stabilizer.available
local pitchHold = PitchHold:new(gimbal)
local pitchSign = SHIP.ATT.invertPitch and -1 or 1

local display = FlightDisplay:new()
local audio = ShipAudio:new(findOptionalPeripherals("speaker"))

-- NAV leg: compass insert is always direct-to. PTN on the display
-- is the only way onto a holding pattern.
local navPattern = false
local lastSteerSource = "WHEEL"

-- Maps the 0-15 throttle lever onto 0 .. SHIP.LNAV.maxSpeed. Detent 0 is stop.
local function leverToTargetVelocity(leverPosition)
    return (leverPosition / MAX_POWER) * SHIP.LNAV.maxSpeed
end

local function controlUpdate()
    -- Compute first, then flush every independent setter in one tick.
    -- Sequential mainThread writes would each cost a server tick.
    local batch = WriteBatch:new()

    local burnerLeverState = burnerLever.getState()
    local landing = burnerLeverState == 0

    if not volumeCaptured then
        altitudeHold:capture(burnerBank:sumTargetAmounts())
        volumeCaptured = true
    end

    local agl, hasGround = readWorstAgl(opticalSensors)
    if not landing then
        landedLatched = false
    elseif isTouchdown(agl, hasGround) then
        landedLatched = true
    end

    local capacity = burnerBank:balloonCapacity()
    local minVolume, maxVolume = burnerBank:volumeLimits(capacity)
    local minAlt = SHIP.VNAV.minAltitude
    local maxAlt = altitudeForHeatedVolume(maxVolume)
    maxAlt = Range:new(minAlt, AltitudeHold.HARD_CEILING):clamp(maxAlt)

    -- Steering first: the mixer is a shared RPM pool, and a hard turn
    -- linearly sheds the speed target so yaw can take the whole engine.
    local steerSource, relativeBearing = selectHeadingCommand(navigationTable, steeringWheel)
    if steerSource == "NAV" and lastSteerSource ~= "NAV" then
        -- Compass just acquired: always Direct To, never a leftover pattern.
        navPattern = false
    elseif steerSource ~= "NAV" then
        navPattern = false
    end
    lastSteerSource = steerSource

    local yawRate = readYawRate(gimbal)
    local navDistance = nil
    local cmdBearing = 0
    local steerRequest
    if steerSource == "NAV" then
        navDistance = readNavDistance(navigationTable)
        cmdBearing = navCommandedBearing(navPattern)
        steerRequest = trackYawRate(
            navYawRateCommand(
                relativeBearing,
                velocitySensor.getVelocity(),
                navDistance,
                cmdBearing
            ),
            yawRate
        )
    else
        steerRequest = dampedSteering(normalizedSteering(relativeBearing), yawRate)
    end

    local leverState = throttleLever.getState()
    local arrived = steerSource == "NAV"
        and not navPattern
        and isFiniteNumber(navDistance)
        and navDistance <= SHIP.LNAV.navArriveRange
    if arrived then
        if leverState ~= 0 then
            local lever = throttleLever
            WriteBatch.defer(batch, function()
                lever.setSignal(0)
            end)
        end
        leverState = 0
    end
    local speedRequest
    if leverState == 0 then
        speedRequest = velocityHold:holdOff()
    else
        local headroom = mixSpeedHeadroom(steerRequest)
        velocityHold:setCeiling(headroom)
        velocityHold:setTarget(leverToTargetVelocity(leverState) * headroom)
        speedRequest = velocityHold:read()
    end
    local mix = thrustMixer:apply(speedRequest, steerRequest, batch)
    local heading = readStandardHeading(navigationTable)

    local targetAltitude = nil
    if not landing then
        targetAltitude = leverToTargetAltitude(burnerLeverState, minAlt, maxAlt, MAX_POWER)
    end

    local vnavMode
    local volume
    if landedLatched then
        -- Hull is on the ground: park heat at the upright floor and
        -- cut props. Stays latched until the lever leaves 0.
        vnavMode = "landed"
        volume = altitudeHold:holdOff(minVolume)
        burnerBank:setTotal(volume, batch)
        verticalPropellers:setSpeed(0, batch)
    else
        volume = altitudeHold:read(
            targetAltitude, minVolume, maxVolume, agl, hasGround, landing, maxAlt
        )
        burnerBank:setTotal(volume, batch)
        if landing then
            vnavMode = hasGround and "flare" or "land"
        elseif altitudeHold.terrain then
            vnavMode = "terrain"
        else
            vnavMode = "hold"
        end
        -- Vertical speed is a consequence of heat-volume slew, not a
        -- prop loop. Park the vertical RSC so leftover VS-boost cannot
        -- keep the hull climbing after the heat command has settled.
        verticalPropellers:setSpeed(0, batch)
    end

    local lnavMode
    if leverState == 0 then
        lnavMode = "stop"
    else
        lnavMode = "hold"
    end

    -- ATT: hold gimbal pitch at 0 via the stabilizer servo. Skip the
    -- loop when the RSC or bearing is missing. Park on the ground so
    -- it does not fight the hull sitting still.
    local attMode
    if not attAvailable then
        attMode = "none"
        pitchHold:sense()
    elseif landedLatched then
        attMode = "off"
        pitchHold:captureState()
        stabilizer:setTarget(0)
        stabilizer:update(batch)
    else
        attMode = "level"
        stabilizer:setTarget(pitchHold:read() * pitchSign)
        stabilizer:update(batch)
    end

    local snapshot = {
        velocity       = velocitySensor.getVelocity(),
        throttle       = leverState,
        targetVelocity = velocityHold.target,
        lnavMode       = lnavMode,
        heading        = heading,
        steerSource    = steerSource,
        navPattern     = navPattern,
        navCommandedBearing = cmdBearing,
        relativeBearing = relativeBearing,
        yawRate        = yawRate,
        requestedSpeed = mix.requestedSpeed,
        requestedSteer = mix.requestedSteer,
        requestedCommonRpm = mix.requestedCommonRpm,
        appliedCommonRpm = mix.appliedCommonRpm,
        speedReduced   = mix.speedReduced,
        propeller1Rpm  = mix.leftRpm,
        propeller2Rpm  = mix.rightRpm,
        altitude       = altitudeHold.lastHeight,
        targetAltitude = targetAltitude,
        landing        = landing,
        vnavMode       = vnavMode,
        verticalSpeed  = altitudeHold.lastVerticalSpeed,
        targetVolume   = altitudeHold.lastTargetVolume,
        currentVolume  = burnerBank.lastAmount,
        agl            = hasGround and agl or nil,
        verticalPropRpm   = verticalPropellers.lastSpeed,
        balloonCapacity = capacity,
        volumeSlewRate = altitudeHold.lastSlewRate,
        altitudeFault  = altitudeHold.fault,
        pitch          = pitchHold.lastPitch,
        pitchRate      = pitchHold.lastPitchRate,
        stabAngle      = stabilizer.lastAngle,
        stabTarget     = stabilizer.target,
        stabRpm        = stabilizer.lastSpeed or 0,
        attMode        = attMode,
        attFault       = pitchHold.fault or (attAvailable and stabilizer.fault),
    }
    display:update(snapshot)
    audio:update(snapshot)
    batch:flush()
end


local timer = os.startTimer(0.1)
while true do
    local event, p1, p2, p3 = os.pullEvent()

    if event == "timer" and p1 == timer then
        controlUpdate()
        timer = os.startTimer(0.1)
    elseif event == "mouse_click" or event == "monitor_touch" then
        -- mouse_click: button, x, y. monitor_touch: side, x, y.
        -- PTN is opt-in only. Direct-to is compass insert, not a click.
        if display:click(p2, p3) == "ptn" and lastSteerSource == "NAV" then
            navPattern = not navPattern
        end
    end
end
