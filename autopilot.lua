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

-- Park the loop: drop the speed target and integral. Detent 0 is stop,
-- not a 0 m/s hold.
function VelocityHold:holdOff()
    self.target = 0
    self.integral:reset()
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

-- LNAV heading helpers. Wheel getTargetAngle() and nav-table getBearing()
-- are the same relative command: 0 straight, +right, -left, wrapped ±180.
-- The mixer does not care which peripheral produced the bearing.

-- Minecraft / nav-table heading (0 = south, ±180) to compass degrees
-- (0 = north, 90 = east, 180 = south, 270 = west), clockwise.
function toStandardHeading(rawHeading)
    if not isFiniteNumber(rawHeading) then
        return nil
    end
    local heading = (180 - rawHeading) % 360
    if heading < 0 then
        heading = heading + 360
    end
    return heading
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

-- VerticalSpeedHold is the VNAV inner loop: vertical-speed error -> burner
-- amount (PI + increase-only slew). Altitude hold, landing flare, and
-- terrain avoidance are outer loops that only write a desiredVS setpoint.
-- Because each burner amount deterministically settles at one altitude, the
-- integral naturally converges on the correct equilibrium amount whenever
-- the commanded rate is zero.
--
-- getVerticalSpeed() is positive when climbing, negative when descending.
VerticalSpeedHold = {}

-- Last-resort override independent of the outer loops: above this world
-- height, force minimum burner amount so the ship cannot push past the
-- sensor/world ceiling (320). MAX_ALTITUDE on AltitudeHold sits below this
-- to leave braking margin for the altitude outer loop.
VerticalSpeedHold.HARD_CEILING = 318

-- Burner amount range comes from the shared BURNER_AMOUNT_RANGE (util.lua) so
-- this controller's clamping/anti-windup always agrees with what BurnerBank
-- (flight.lua) actually sends to the peripherals.
VerticalSpeedHold.RATE_P_GAIN  = 40.0
VerticalSpeedHold.RATE_I_GAIN  = 10.0
VerticalSpeedHold.RATE_I_LIMIT = 200.0

-- Slew limit: burner amount may increase at most this fast (per second) to
-- avoid abrupt heat spikes that overshoot; decreases are never slew-limited
-- since cutting heat is the safe direction.
VerticalSpeedHold.MAX_INCREASE_RATE = 150.0

local function isValidHeight(h)
    return type(h) == "number" and h == h and h > -1000 and h < 1000
end

local function isValidRate(v)
    return type(v) == "number" and v == v
end

function VerticalSpeedHold:new(altitudeSensor)
    local t = setmetatable({}, { __index = VerticalSpeedHold })
    t.sensor     = altitudeSensor
    t.integral   = ClampedIntegral:new(VerticalSpeedHold.RATE_I_LIMIT)
    t.lastAmount = BURNER_AMOUNT_RANGE.min
    t.lastProposed = BURNER_AMOUNT_RANGE.min
    t.lastRateError = 0
    t.lastClock  = nil
    t.fault      = false
    t.slewLimited = false
    t.lastHeight = nil
    t.lastVerticalSpeed = nil
    return t
end

-- Seed the integral from a known current burner amount for bumpless startup
-- (e.g. when the control loop first engages). At the capture moment we don't
-- know the actual rate error, so this assumes it is near zero.
function VerticalSpeedHold:capture(currentAmount)
    if currentAmount ~= nil then
        self.integral:set(currentAmount / VerticalSpeedHold.RATE_I_GAIN)
    end
    self.lastClock = nil  -- force dt recalibration on next read
end

-- Returns a burner amount clamped to BURNER_AMOUNT_RANGE. Call once per tick;
-- this is stateful (integral, slew, dt) like VelocityHold:read().
function VerticalSpeedHold:read(desiredVS)
    local height = self.sensor.getHeight()
    local verticalSpeed = self.sensor.getVerticalSpeed()

    local dt = stepClock(self, 0.1)

    if not isValidHeight(height) or not isValidRate(verticalSpeed) then
        -- Sensor fault: freeze the last commanded amount rather than
        -- integrating on bad data or guessing a new one. Zero the rate
        -- error so the vertical props do not keep boosting on stale data.
        self.fault = true
        self.lastRateError = 0
        self.slewLimited = false
        return self.lastAmount
    end
    self.fault = false
    self.lastHeight = height
    self.lastVerticalSpeed = verticalSpeed

    -- Hard override: independent of target/controller, never allow the
    -- computed output to push past the ceiling margin. Cut props too.
    if height >= VerticalSpeedHold.HARD_CEILING then
        self.integral:set(math.min(self.integral.value, 0))
        self.lastAmount = BURNER_AMOUNT_RANGE.min
        self.lastProposed = BURNER_AMOUNT_RANGE.min
        self.lastRateError = 0
        self.slewLimited = false
        return self.lastAmount
    end

    local rateError = desiredVS - verticalSpeed
    local proposed = (rateError * VerticalSpeedHold.RATE_P_GAIN)
        + ((self.integral.value + rateError * dt) * VerticalSpeedHold.RATE_I_GAIN)
    self.lastProposed = proposed
    self.lastRateError = rateError

    -- Conditional anti-windup: only accumulate if doing so wouldn't push the
    -- output further past a limit it has already saturated against.
    local willSaturateHigh = proposed > BURNER_AMOUNT_RANGE.max and rateError > 0
    local willSaturateLow  = proposed < BURNER_AMOUNT_RANGE.min and rateError < 0
    if not (willSaturateHigh or willSaturateLow) then
        self.integral:add(rateError * dt)
    end

    local amount = (rateError * VerticalSpeedHold.RATE_P_GAIN)
        + (self.integral.value * VerticalSpeedHold.RATE_I_GAIN)
    amount = BURNER_AMOUNT_RANGE:clamp(amount)

    -- Slew limit increases only; decreases apply immediately for safety.
    -- slewLimited is true when heat could not follow the PI this tick
    -- (slew cap or high saturation), so leftover climb can go to the props.
    self.slewLimited = willSaturateHigh
    if amount > self.lastAmount then
        local slewed = math.min(amount, self.lastAmount + VerticalSpeedHold.MAX_INCREASE_RATE * dt)
        if slewed < amount then
            self.slewLimited = true
        end
        amount = slewed
    end

    self.lastAmount = amount
    return amount
end

-- Park the inner loop: sample sensors for the display, command minimum
-- heat, and seed the integral so a later takeoff does not slam from a
-- stale hover amount. Used once the hull is on the ground.
function VerticalSpeedHold:holdOff()
    local height = self.sensor.getHeight()
    local verticalSpeed = self.sensor.getVerticalSpeed()
    if isValidHeight(height) then
        self.lastHeight = height
    end
    if isValidRate(verticalSpeed) then
        self.lastVerticalSpeed = verticalSpeed
    end
    self.fault = not (isValidHeight(height) and isValidRate(verticalSpeed))
    self.lastRateError = 0
    self.slewLimited = false
    self.lastAmount = BURNER_AMOUNT_RANGE.min
    self.lastProposed = BURNER_AMOUNT_RANGE.min
    self.integral:set(BURNER_AMOUNT_RANGE.min / VerticalSpeedHold.RATE_I_GAIN)
    self.lastClock = nil
    return self.lastAmount
end

-- AltitudeHold is the cruise outer loop: altitude error -> desired vertical
-- speed (clamped, asymmetric). It does not command burners; VerticalSpeedHold
-- tracks the rate it produces.
AltitudeHold = {}

-- Operational altitude range. MAX_ALTITUDE is deliberately below the true
-- sensor/world ceiling (320) to leave braking margin; the last-resort cut
-- is VerticalSpeedHold.HARD_CEILING.
AltitudeHold.MIN_ALTITUDE = 60
AltitudeHold.MAX_ALTITUDE = 315

-- Outer loop: metres of desired vertical speed per metre of altitude error,
-- clamped asymmetrically since burners have far more climb authority than
-- descend authority (descending just means less heat, not active cooling).
AltitudeHold.APPROACH_GAIN    = 0.15
AltitudeHold.MAX_CLIMB_RATE   = 3.0   -- m/s, tune in-game
AltitudeHold.MAX_DESCENT_RATE = 2.0   -- m/s, tune in-game

-- Altitude errors within this margin command zero desired rate instead of a
-- tiny nonzero one. Without this, sensor/physics noise near the setpoint
-- keeps commanding a small trickling rate, which the (well-tuned) rate loop
-- faithfully tracks, causing a slow hunt/oscillation around the target. The
-- error is "shrunk" by the deadband rather than hard-zeroed outside it, so
-- desiredRate stays continuous at the band edge (no new discontinuity).
AltitudeHold.DEADBAND = 3.0   -- metres, tune in-game

function AltitudeHold:new()
    local t = setmetatable({}, { __index = AltitudeHold })
    t.target = AltitudeHold.MIN_ALTITUDE
    t.lastDesiredRate = 0
    return t
end

-- Set an explicit target altitude, clamped to the operational range.
-- Deliberately does NOT reset the VS-hold integral: the target changes
-- continuously as the lever moves, and the integral represents the burner
-- amount the inner loop has already found for the current regime.
function AltitudeHold:setTarget(altitude)
    self.target = Range:new(AltitudeHold.MIN_ALTITUDE, AltitudeHold.MAX_ALTITUDE):clamp(altitude)
end

-- Outer loop only: altitude error -> desired vertical speed.
function AltitudeHold:desiredRate(height)
    if not isValidHeight(height) then
        return self.lastDesiredRate
    end
    local altError = self.target - height
    local shrunkError = 0
    if altError > AltitudeHold.DEADBAND then
        shrunkError = altError - AltitudeHold.DEADBAND
    elseif altError < -AltitudeHold.DEADBAND then
        shrunkError = altError + AltitudeHold.DEADBAND
    end
    local desiredRate = shrunkError * AltitudeHold.APPROACH_GAIN
    desiredRate = Range:new(-AltitudeHold.MAX_DESCENT_RATE, AltitudeHold.MAX_CLIMB_RATE):clamp(desiredRate)
    self.lastDesiredRate = desiredRate
    return desiredRate
end

-- VNAV outer-loop helpers shared by landing flare and cruise terrain
-- avoidance. Optical sensors never drive an actuator directly; they only
-- shape the desiredVS that VerticalSpeedHold tracks.
VNav = {}

VNav.CLEARANCE              = 12.0  -- metres AGL; well outside AltitudeHold.DEADBAND
VNav.LANDING_APPROACH_SINK  = -AltitudeHold.MAX_DESCENT_RATE  -- m/s while no optical hit
VNav.LANDING_SINK           = -1.0  -- m/s at first contact; flare starts here
VNav.LANDING_SETTLE_SINK    = -0.2  -- m/s held until latch; 0 at settle would hover above it
VNav.TOUCHDOWN_MARGIN       = 0.3   -- metres; latch a little above the rest reading
VNav.OPTICAL_RANGE          = 15.0  -- metres; flare starts from first contact / this range
VNav.PROP_DEADBAND  = 0.2   -- m/s; ignore tiny rate errors so props stay off at hover
VNav.PROP_GAIN      = 256 / 3  -- RPM per m/s of leftover climb (3 m/s -> hardware max)

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
    local climb = (VNav.CLEARANCE - agl) * AltitudeHold.APPROACH_GAIN
    return Range:new(0, AltitudeHold.MAX_CLIMB_RATE):clamp(climb)
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
