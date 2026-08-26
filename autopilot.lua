require("util")
require("config")

-- CC peripherals may return a list or multiple values. Accept either.
local function unpackReading(first, second, third)
    if type(first) == "table" then
        return first[1], first[2], first[3]
    end
    return first, second, third
end

-- VelocityHold maintains a target velocity and computes a PI control
-- output as a normalized speed request in [0, 1]. The differential
-- mixer (flight.lua) scales that request by SHIP.LNAV.forwardRpm.
VelocityHold = {}

-- Analog loop was 15 power per 1 m/s of error (full scale). Same
-- fraction of the normalized range: 1 m/s commands 1.0.
VelocityHold.GAIN    = 1.0
VelocityHold.I_GAIN  = 2.0 / 15.0  -- same I/P ratio as the analog / RPM loops
VelocityHold.I_LIMIT = 15.0        -- accumulated velocity-error ticks; not RPM

function VelocityHold:new(velocitySensor, minOutput, maxOutput)
    local t = setmetatable({}, { __index = VelocityHold })
    t.sensor   = velocitySensor
    t.target   = 0
    t.integral = ClampedIntegral:new(VelocityHold.I_LIMIT)
    t.output   = Range:new(minOutput or 0, maxOutput or 1)
    return t
end

-- Set an explicit target velocity. Deliberately does NOT reset the integral:
-- the throttle lever changes continuously, and the integral is the command
-- the controller has already found. Resetting on every notch would bump output.
function VelocityHold:setTarget(velocity)
    self.target = velocity
end

-- Nudge the target velocity by a small amount without resetting the integral.
-- Use this for incremental adjustments (e.g. button presses) to avoid sudden
-- control changes.
function VelocityHold:nudgeTarget(delta)
    self.target = self.target + delta
end

-- Cap the normalized output. Used each tick so a hard turn can steal
-- RPM without the integral winding against a mixer limit the loop
-- cannot see. Pass 1 to restore the full engine.
function VelocityHold:setCeiling(maxOutput)
    if not isFiniteNumber(maxOutput) then
        maxOutput = 1
    end
    self.output.max = Range:new(0, 1):clamp(maxOutput)
    if self.output.max < self.output.min then
        self.output.max = self.output.min
    end
end

-- Park the loop: drop the speed target and integral. Detent 0 is stop,
-- not a 0 m/s hold.
function VelocityHold:holdOff()
    self.target = 0
    self.integral:reset()
    self:setCeiling(1)
    return 0
end

-- Capture the current velocity as the target (used when engaging hold mode).
-- Optionally pass the current normalized command to seed the integral so
-- output starts smoothly from the current request rather than from zero.
function VelocityHold:captureTarget(currentNormalized)
    self:setTarget(self.sensor.getVelocity())
    if currentNormalized ~= nil then
        -- Back-calculate integral so initial output matches current command.
        -- At capture moment error is 0, so output = integral * I_GAIN.
        self.integral:set(currentNormalized / VelocityHold.I_GAIN)
    end
end

-- Returns a PI speed request in [0, 1] based on velocity error.
function VelocityHold:read()
    local error = self.target - self.sensor.getVelocity()

    -- Conditional anti-windup: don't keep accumulating into a limit the
    -- output has already saturated against, or the integral has to unwind
    -- before the ship responds to the next lever change.
    local proposed = (error * VelocityHold.GAIN)
        + ((self.integral.value + error) * VelocityHold.I_GAIN)
    local saturatedHigh = proposed > self.output.max and error > 0
    local saturatedLow  = proposed < self.output.min and error < 0
    if not (saturatedHigh or saturatedLow) then
        self.integral:add(error)
    end

    local value = (error * VelocityHold.GAIN) + (self.integral.value * VelocityHold.I_GAIN)
    value = self.output:clamp(value)
    return value
end

-- LNAV heading helpers. Wheel getTargetAngle() is a deflection. The
-- nav table's getBearing() is a compass error to the resolved target
-- (lodestone / spawn / map). Both are 0 straight, +right, -left, ±180.
-- Wheel maps through normalizedSteering; NAV commands a yaw rate so
-- cruise can keep up with a swinging needle.

-- Wrap into [0, 360).
function wrapDegrees360(angle)
    if not isFiniteNumber(angle) then
        return nil
    end
    angle = angle % 360
    if angle < 0 then
        angle = angle + 360
    end
    return angle
end

-- Wrap into (-180, 180].
function wrapDegrees180(angle)
    angle = wrapDegrees360(angle)
    if angle == nil then
        return nil
    end
    if angle > 180 then
        angle = angle - 360
    end
    return angle
end

local function navTableYaw()
    local yaw = SHIP.LNAV.navTableYaw
    if not isFiniteNumber(yaw) then
        return 0
    end
    return yaw
end

-- Minecraft / nav-table heading (0 = south, ±180) to compass degrees
-- (0 = north, 90 = east, 180 = south, 270 = west), clockwise, then
-- rotated by SHIP.LNAV.navTableYaw so the table's 0-mark is north.
function toStandardHeading(rawHeading)
    if not isFiniteNumber(rawHeading) then
        return nil
    end
    return wrapDegrees360((180 - rawHeading) + navTableYaw())
end

function readStandardHeading(navigationTable)
    if navigationTable == nil then
        return nil
    end
    return toStandardHeading(unpackReading(navigationTable.getHeading()))
end

-- NAV whenever the table has a live target; otherwise WHEEL.
-- Returns source ("NAV"|"WHEEL"), relative bearing (degrees).
function selectHeadingCommand(navigationTable, steeringWheel)
    if navigationTable ~= nil and navigationTable.hasTarget() then
        local bearing = unpackReading(navigationTable.getBearing())
        if not isFiniteNumber(bearing) then
            bearing = 0
        end
        bearing = wrapDegrees180(bearing + navTableYaw())
        return "NAV", bearing
    end
    local angle = 0
    if steeringWheel ~= nil then
        angle = unpackReading(steeringWheel.getTargetAngle())
        if not isFiniteNumber(angle) then
            angle = 0
        end
    end
    return "WHEEL", angle
end

-- Gimbal yaw rate about body-Y (deg/s). Nil when the sensor is
-- missing or the reading is unusable; the damper then falls open-loop.
function readYawRate(gimbal)
    if gimbal == nil then
        return nil
    end
    local _wx, wy = unpackReading(gimbal.getAngularRates())
    if not isFiniteNumber(wy) then
        return nil
    end
    return wy
end

-- Linear relative bearing -> normalized steering in [-1, 1].
-- Deadband and full-authority angle come from SHIP.LNAV.
function normalizedSteering(bearing)
    if not isFiniteNumber(bearing) then
        return 0
    end
    local deadband = SHIP.LNAV.steerDeadband
    local fullAngle = SHIP.LNAV.steerFullAngle
    local magnitude = math.abs(bearing)
    if magnitude <= deadband then
        return 0
    end
    local span = fullAngle - deadband
    local command
    if span <= 0 then
        command = 1
    else
        command = (magnitude - deadband) / span
        if command > 1 then
            command = 1
        end
    end
    if bearing < 0 then
        return -command
    end
    return command
end

-- Right-turn-positive yaw rate with yawDampDeadband applied. Nil
-- when the gimbal reading is unusable so callers can fall open-loop.
-- Body +wy is a left turn (CCW about up); +steer is a right turn, so
-- the measured rate is flipped unless invertYawDamp is set.
function measuredYawRate(yawRate)
    if not isFiniteNumber(yawRate) then
        return nil
    end
    local measured = -yawRate
    if SHIP.LNAV.invertYawDamp then
        measured = yawRate
    end
    local deadband = SHIP.LNAV.yawDampDeadband
    if math.abs(measured) <= deadband then
        return 0
    elseif measured > 0 then
        return measured - deadband
    end
    return measured + deadband
end

-- Track a yaw-rate setpoint (deg/s, right-turn positive). Equilibrium
-- is at yawCmd even when |yawCmd| > 1/yawDampGain; the mixer saturates
-- until the hull catches up. That is what lets NAV turn faster at
-- cruise without weakening the stop-turn brake. No gimbal: map the
-- rate back to a thrust request with the same gain.
function trackYawRate(yawCmd, yawRate)
    local gain = SHIP.LNAV.yawDampGain
    if not isFiniteNumber(yawCmd) then
        yawCmd = 0
    end
    local measured = measuredYawRate(yawRate)
    if measured == nil then
        return Range:new(-1, 1):clamp(yawCmd * gain)
    end
    return Range:new(-1, 1):clamp((yawCmd - measured) * gain)
end

-- Blend a normalized wheel request with the yaw-rate damper. Full
-- deflection settles at about 1/yawDampGain deg/s. At a request of 0
-- this is a stop-turn loop. No gimbal -> returns the request.
function dampedSteering(steerReq, yawRate)
    steerReq = Range:new(-1, 1):clamp(steerReq or 0)
    local gain = SHIP.LNAV.yawDampGain
    if gain <= 0 then
        return steerReq
    end
    return trackYawRate(steerReq / gain, yawRate)
end

-- Horizontal range to the nav-table target (metres). 3D distance with
-- the vertical offset removed: sqrt(d^2 - dy^2). Nil when the table
-- is missing or the 3D reading is unusable. Falls back to 3D if the
-- vertical offset is absent.
-- https://solastrius.github.io/CreateAvionics/peripheral/navigation_table.html
function readNavDistance(navigationTable)
    if navigationTable == nil or navigationTable.getDistanceToTarget == nil then
        return nil
    end
    local distance = unpackReading(navigationTable.getDistanceToTarget())
    if not isFiniteNumber(distance) or distance < 0 then
        return nil
    end
    if navigationTable.getVerticalOffsetToTarget == nil then
        return distance
    end
    local dy = unpackReading(navigationTable.getVerticalOffsetToTarget())
    if not isFiniteNumber(dy) then
        return distance
    end
    local horizontalSq = distance * distance - dy * dy
    if horizontalSq <= 0 then
        return 0
    end
    return math.sqrt(horizontalSq)
end

-- Relative bearing the NAV tracker should fly. Direct-to is 0 (nose
-- on the target). A holding pattern is ±90° (target abeam) so the
-- hull orbits at whatever radius speed and turn rate produce.
function navCommandedBearing(pattern)
    if not pattern then
        return 0
    end
    if SHIP.LNAV.navHoldClockwise == false then
        return -90
    end
    return 90
end

-- Commanded yaw rate (deg/s) that tracks commandedBearing. Direct-to
-- uses 0 (nose on the compass). A holding pattern uses ~±90° so the
-- hull orbits. P is on bearing error; LOS uses the actual needle so
-- a tangent orbit matches v/range.
function navYawRateCommand(bearing, speed, distance, commandedBearing)
    if not isFiniteNumber(bearing) then
        return 0
    end
    if not isFiniteNumber(commandedBearing) then
        commandedBearing = 0
    end
    local trackError = wrapDegrees180(bearing - commandedBearing)
    if not isFiniteNumber(trackError) then
        trackError = 0
    end

    local bearingTerm = 0
    local deadband = SHIP.LNAV.steerDeadband
    local magnitude = math.abs(trackError)
    if magnitude > deadband then
        local signed = trackError
        if trackError > 0 then
            signed = trackError - deadband
        else
            signed = trackError + deadband
        end
        bearingTerm = signed * SHIP.LNAV.navBearingGain
    end

    local losTerm = 0
    if isFiniteNumber(speed) and isFiniteNumber(distance) then
        local range = distance
        local minRange = SHIP.LNAV.navMinRange
        if range < minRange then
            range = minRange
        end
        if range > 0 then
            losTerm = math.deg(speed * math.sin(math.rad(bearing)) / range)
                * SHIP.LNAV.navLosGain
        end
    end

    local yawCmd = bearingTerm + losTerm
    local cruiseRate = SHIP.LNAV.navMaxYawRate
    local slowRate = cruiseRate
    local gain = SHIP.LNAV.yawDampGain
    if gain > 0 then
        slowRate = 1 / gain
    end
    local maxRate = cruiseRate
    if isFiniteNumber(speed) and SHIP.LNAV.maxSpeed > 0 then
        local t = Range:new(0, 1):clamp(math.abs(speed) / SHIP.LNAV.maxSpeed)
        maxRate = slowRate + t * (cruiseRate - slowRate)
    end
    return Range:new(-maxRate, maxRate):clamp(yawCmd)
end

-- VNAV constants and the altitude-hold inner loop.
-- Optical sensors never drive an actuator; a hasHit() while dumping
-- heat switches the slew to SHIP.VNAV.landingVolumeRate. Takeoff (heat
-- increasing) and cruise with no ground contact lerp minVolumeRate ..
-- maxVolumeRate against minAltitude .. max altitude.
VNav = {}

VNav.SEA_LEVEL              = 63    -- world Y; matches the mod's Overworld sea level
VNav.ALTITUDE_DECAY         = 250   -- metres for a 1/e lift drop (mod pressure curve)
VNav.LIFT_PER_CUBIC_METER   = 15.63 -- pN of lift per m³ of heated volume at sea level
VNav.CLEARANCE              = 12.0  -- metres AGL
VNav.TERRAIN_GAIN           = 0.15  -- m/s per metre below CLEARANCE
VNav.MAX_CLIMB_RATE         = 3.0   -- m/s, leftover prop boost cap (unused by volume slew)
VNav.LANDING_APPROACH_SINK  = -2.0  -- m/s while no optical hit (display/docs)
VNav.LANDING_SINK           = -1.0  -- m/s at first contact; flare starts here
VNav.LANDING_SETTLE_SINK    = -0.2  -- m/s held until latch; 0 at settle would hover above it
VNav.TOUCHDOWN_MARGIN       = 0.3   -- metres; latch a little above the rest reading
VNav.OPTICAL_RANGE          = 15.0  -- metres; flare starts from first contact / this range
VNav.PROP_DEADBAND  = 0.2   -- m/s; ignore tiny rate errors so props stay off at hover
VNav.PROP_GAIN      = 256 / 3  -- RPM per m/s of leftover climb (3 m/s -> hardware max)

-- Heated-volume formula (Create Aeronautics lift curve). Equilibrium
-- altitude H needs V = F_down * e^((H - sea) / decay) / lift_per_m3.
-- Inverse: H = sea + decay * ln(V * lift_per_m3 / F_down).
--
-- AltitudeHold is the VNAV inner loop: target Y -> target heated volume
-- via the formula, then the commanded volume slews toward that target.
-- Vertical speed is a consequence of that slew rate, not a control input.
-- getVerticalSpeed() is sampled only for the display.
--
-- Slew is faster at high world Y and slower down low (lerp of
-- minVolumeRate .. maxVolumeRate across minAltitude .. max altitude).
-- An optical hasHit() while descending uses landingVolumeRate; takeoff
-- and lever 0 without a hit keep the altitude lerp.

local function isValidHeight(h)
    return type(h) == "number" and h == h and h > -1000 and h < 1000
end

local function isValidRate(v)
    return type(v) == "number" and v == v
end

function heatedVolumeForAltitude(height)
    local fDown = SHIP.VNAV.downwardForce
    return fDown * math.exp((height - VNav.SEA_LEVEL) / VNav.ALTITUDE_DECAY)
        / VNav.LIFT_PER_CUBIC_METER
end

function altitudeForHeatedVolume(volume)
    local fDown = SHIP.VNAV.downwardForce
    local ratio = volume * VNav.LIFT_PER_CUBIC_METER / fDown
    if not (ratio > 0) then
        return VNav.SEA_LEVEL
    end
    return VNav.SEA_LEVEL + VNav.ALTITUDE_DECAY * math.log(ratio)
end

-- Lever 0 is land (nil target). Notches 1 .. maxPower lerp minAlt .. maxAlt.
function leverToTargetAltitude(leverPosition, minAlt, maxAlt, maxPower)
    maxPower = maxPower or 15
    if leverPosition == nil or leverPosition <= 0 then
        return nil
    end
    if maxAlt < minAlt then
        maxAlt = minAlt
    end
    if leverPosition >= maxPower then
        return maxAlt
    end
    local t = (leverPosition - 1) / (maxPower - 1)
    return minAlt + t * (maxAlt - minAlt)
end

AltitudeHold = {}

-- Last-resort override independent of the outer loops: above this world
-- height, force minimum volume so the ship cannot push past the
-- sensor/world ceiling (320).
AltitudeHold.HARD_CEILING = 318

function AltitudeHold:new(altitudeSensor)
    local t = setmetatable({}, { __index = AltitudeHold })
    t.sensor = altitudeSensor
    t.lastVolume = nil
    t.lastTargetVolume = nil
    t.lastSlewRate = nil
    t.lastClock = nil
    t.fault = false
    t.slewLimited = false
    t.terrain = false
    t.lastHeight = nil
    t.lastVerticalSpeed = nil
    return t
end

-- Seed commanded volume from the current burner total so the first
-- slew step is bumpless. Forces dt recalibration on the next read.
function AltitudeHold:capture(currentVolume)
    if currentVolume ~= nil then
        self.lastVolume = currentVolume
    end
    self.lastClock = nil
end

-- m³/s the heat command may move. Cruise (and takeoff): lerp
-- minVolumeRate at minAltitude to maxVolumeRate at maxAlt. An optical
-- hasHit() while dumping heat uses landingVolumeRate.
local function volumeSlewRate(height, hasGround, maxAlt, descending)
    if hasGround and descending then
        return SHIP.VNAV.landingVolumeRate
    end
    local minAlt = SHIP.VNAV.minAltitude
    maxAlt = maxAlt or minAlt
    local span = maxAlt - minAlt
    local t = 0
    if span > 0 then
        t = Range:new(0, 1):clamp((height - minAlt) / span)
    end
    return SHIP.VNAV.minVolumeRate
        + t * (SHIP.VNAV.maxVolumeRate - SHIP.VNAV.minVolumeRate)
end

local function clampVolume(volume, minVolume, maxVolume)
    if volume < minVolume then
        return minVolume
    end
    if volume > maxVolume then
        return maxVolume
    end
    return volume
end

-- Returns a total heated-volume command. Call once per tick; this is
-- stateful (slew, dt) like VelocityHold:read().
-- targetAltitude is nil while landing (slew toward minVolume).
function AltitudeHold:read(targetAltitude, minVolume, maxVolume, agl, hasGround, landing, maxAlt)
    local height = self.sensor.getHeight()
    local verticalSpeed = self.sensor.getVerticalSpeed()

    local dt = stepClock(self, 0.1)
    if self.lastVolume == nil then
        self.lastVolume = minVolume
    end

    if isValidRate(verticalSpeed) then
        self.lastVerticalSpeed = verticalSpeed
    end

    if not isValidHeight(height) then
        -- Sensor fault: freeze the last commanded volume rather than
        -- slewing on a bad height. VS is display-only and does not fault.
        self.fault = true
        self.slewLimited = false
        self.terrain = false
        self.lastSlewRate = 0
        return self.lastVolume
    end
    self.fault = false
    self.lastHeight = height

    -- Hard override: independent of target/controller, never allow the
    -- computed output to push past the ceiling margin.
    if height >= AltitudeHold.HARD_CEILING then
        self.lastTargetVolume = minVolume
        self.lastVolume = minVolume
        self.slewLimited = false
        self.terrain = false
        self.lastSlewRate = 0
        return self.lastVolume
    end

    local wantAltitude = targetAltitude
    self.terrain = false
    if not landing and hasGround and agl < VNav.CLEARANCE then
        local clearanceAlt = height + (VNav.CLEARANCE - agl)
        if wantAltitude == nil or clearanceAlt > wantAltitude then
            wantAltitude = clearanceAlt
        end
        self.terrain = true
    end

    local wantVolume
    if landing or wantAltitude == nil then
        wantVolume = minVolume
    else
        wantVolume = heatedVolumeForAltitude(wantAltitude)
    end
    wantVolume = clampVolume(wantVolume, minVolume, maxVolume)
    self.lastTargetVolume = wantVolume

    local rate = volumeSlewRate(
        height, hasGround, maxAlt, wantVolume < self.lastVolume
    )
    self.lastSlewRate = rate
    local maxStep = rate * dt
    local delta = wantVolume - self.lastVolume
    local slewed = delta
    if delta > maxStep then
        slewed = maxStep
    elseif delta < -maxStep then
        slewed = -maxStep
    end
    self.slewLimited = math.abs(delta) > maxStep + 0.0001

    self.lastVolume = self.lastVolume + slewed
    return self.lastVolume
end

-- Park the inner loop: sample sensors for the display, command the
-- upright-floor heat, and clear slew state so a later takeoff does not
-- slam from a stale cruise volume. Used once the hull is on the ground.
function AltitudeHold:holdOff(minVolume)
    local height = self.sensor.getHeight()
    local verticalSpeed = self.sensor.getVerticalSpeed()
    if isValidHeight(height) then
        self.lastHeight = height
    end
    if isValidRate(verticalSpeed) then
        self.lastVerticalSpeed = verticalSpeed
    end
    self.fault = not isValidHeight(height)
    self.slewLimited = false
    self.terrain = false
    self.lastVolume = minVolume
    self.lastTargetVolume = minVolume
    self.lastSlewRate = 0
    self.lastClock = nil
    return self.lastVolume
end

-- VNAV outer-loop helpers shared by landing flare and cruise terrain
-- avoidance. An optical hasHit() while descending switches AltitudeHold
-- to landingVolumeRate; these helpers only shape target altitude / latch.

-- Worst-case AGL: minimum getDistance() among sensors that hasHit().
-- A miss means "beyond range", never 0. Returns agl, hasGround.
function readWorstAgl(sensors)
    local agl = nil
    for _, sensor in ipairs(sensors) do
        if sensor ~= nil and sensor.hasHit() then
            local distance = sensor.getDistance()
            if type(distance) == "number" and distance == distance then
                if agl == nil or distance < agl then
                    agl = distance
                end
            end
        end
    end
    if agl == nil then
        return nil, false
    end
    return agl, true
end

-- Ship-measured hull-on-ground AGL; see SHIP.VNAV.touchdownAgl in config.lua.
local function touchdownAgl()
    return SHIP.VNAV.touchdownAgl
end

-- True once a downward sensor reports AGL at or below the measured
-- hull-on-ground height (plus a small margin for sensor/hover offset).
function isTouchdown(agl, hasGround)
    return hasGround and agl <= touchdownAgl() + VNav.TOUCHDOWN_MARGIN
end

-- Lever 0: fast sink until ground contact, then linear flare from
-- LANDING_SINK at first contact (~OPTICAL_RANGE) toward settle. Never
-- commands 0 before latch: a zero DVS at touchdownAgl just hovers there.
function landingDesiredVS(agl, hasGround)
    if not hasGround then
        return VNav.LANDING_APPROACH_SINK
    end
    local settle = touchdownAgl()
    local span = VNav.OPTICAL_RANGE - settle
    if span <= 0 then
        return VNav.LANDING_SETTLE_SINK
    end
    local t = (agl - settle) / span
    t = Range:new(0, 1):clamp(t)
    local desired = VNav.LANDING_SINK * t
    if desired > VNav.LANDING_SETTLE_SINK then
        desired = VNav.LANDING_SETTLE_SINK
    end
    return desired
end

-- Cruise terrain override: climb demand when worst-case AGL is below
-- CLEARANCE. Returns 0 when there is no hit or AGL is at/above clearance.
function terrainClimbVS(agl, hasGround)
    if not hasGround or agl >= VNav.CLEARANCE then
        return 0
    end
    local climb = (VNav.CLEARANCE - agl) * VNav.TERRAIN_GAIN
    return Range:new(0, VNav.MAX_CLIMB_RATE):clamp(climb)
end

-- No-integral leftover boost as RSC RPM. Positive rateError only;
-- negative error (need more sink) never spins the vertical props.
function verticalPropRpm(rateError, slewLimited, hasGround)
    if rateError == nil or rateError <= VNav.PROP_DEADBAND then
        return 0
    end
    if not (slewLimited or hasGround) then
        return 0
    end
    return Range:new(0, SHIP.VNAV.maxRpm):clamp(rateError * VNav.PROP_GAIN)
end

-- PitchHold is the attitude outer loop: gimbal pitch error -> desired
-- stabilizer angle. The Servo (flight.lua) is the only thing that talks
-- to the rotational speed controller; this just writes a setpoint.
-- invertPitch in SHIP.ATT is applied by the caller if a positive
-- stabilizer angle pitches the hull the wrong way.
PitchHold = {}

PitchHold.GAIN     = 6.0    -- stabilizer deg per deg of pitch outside the deadband
PitchHold.D_GAIN   = 0.4    -- stabilizer deg per deg/s of pitch rate (damping)
PitchHold.I_GAIN   = 2.0    -- stabilizer deg per degree-second; 2° residual ~4°/s of trim
PitchHold.I_LIMIT  = 22.5   -- degree-seconds; I term max is I_LIMIT * I_GAIN (45)
PitchHold.I_BAND   = 10.0   -- degrees; only integrate near level
PitchHold.DEADBAND = 0.3    -- degrees; P drops to 0 here, I holds the trim angle

function PitchHold:new(gimbal)
    local t = setmetatable({}, { __index = PitchHold })
    t.gimbal = gimbal
    t.integral = ClampedIntegral:new(PitchHold.I_LIMIT)
    t.travel = Range:new(SHIP.ATT.minAngle, SHIP.ATT.maxAngle)
    t.lastPitch = nil
    t.lastPitchRate = 0
    t.lastOutput = 0
    t.fault = false
    t.lastClock = nil
    return t
end

function PitchHold:captureState()
    self.integral:reset()
    self.lastClock = nil
end

-- Sample gimbal pitch and rate into lastPitch / lastPitchRate.
-- Returns true when the reading is usable. Does not run the hold loop,
-- so it is safe when the stabilizer is not installed.
function PitchHold:sense()
    if self.gimbal == nil then
        self.fault = true
        return false
    end

    local pitch, _roll = unpackReading(self.gimbal.getAngles())
    local wx = unpackReading(self.gimbal.getAngularRates())
    if not isFiniteNumber(pitch) then
        self.fault = true
        return false
    end
    self.fault = false
    self.lastPitch = pitch
    if isFiniteNumber(wx) then
        self.lastPitchRate = wx
    end
    return true
end

-- Returns a desired stabilizer angle clamped to SHIP.ATT travel.
-- Call once per tick; the integral is stateful.
function PitchHold:read()
    if not self:sense() then
        return self.lastOutput
    end

    local pitch = self.lastPitch
    local wx = self.lastPitchRate

    local dt = stepClock(self, 0.1)

    -- Target is level (0), same-sign command. A leftover pitch is almost
    -- always a trim problem: the hull needs a nonzero stab angle at
    -- equilibrium. Zeroing the command (or the integral) inside the
    -- deadband dumps that trim and the pitch walks right back out.
    -- Shrink P to 0 across the band so the edge is continuous; freeze I
    -- inside the band and keep integrating a residual until we get there.
    local shrunk = 0
    if pitch > PitchHold.DEADBAND then
        shrunk = pitch - PitchHold.DEADBAND
    elseif pitch < -PitchHold.DEADBAND then
        shrunk = pitch + PitchHold.DEADBAND
    end

    if math.abs(pitch) > PitchHold.I_BAND then
        self.integral:reset()
    elseif math.abs(pitch) > PitchHold.DEADBAND then
        self.integral:add(pitch * dt)
    end

    local value = (shrunk * PitchHold.GAIN)
        + (self.integral.value * PitchHold.I_GAIN)
        + (wx * PitchHold.D_GAIN)
    value = self.travel:clamp(value)
    self.lastOutput = value
    return value
end
