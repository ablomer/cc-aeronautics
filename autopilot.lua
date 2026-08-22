require("util")
require("config")

-- BearingHold steers the ship towards a navigation target by computing the
-- bearing error and producing a PID steering output in [-1, 1].
-- The D term is what keeps the hull from swinging past the target at speed:
-- proportional alone commands full lock right up to the zero crossing, and
-- the ship still carries yaw rate through it.
BearingHold = {}

BearingHold.GAIN       = 1 / 45    -- proportional; maps ±45° error to ±1.0 (10° → ~0.22)
BearingHold.D_GAIN     = 0.035     -- output per degree/second of closing rate (damping);
                                   -- raise if it still overshoots, lower if it crawls in
BearingHold.D_SMOOTH   = 0.4       -- rate low-pass weight; the bearing reading is coarse,
                                   -- so an unfiltered derivative chatters at this gain
BearingHold.I_GAIN     = 0.02      -- output per degree-second of accumulated error
BearingHold.I_LIMIT    = 10.0      -- degree-seconds; I term max is I_LIMIT * I_GAIN (0.2)
BearingHold.I_BAND     = 20.0      -- degrees; only integrate near the target, so big
                                   -- turns cannot wind up and then drive an overshoot
BearingHold.MIN_OUTPUT = 0.15      -- minimum effective output magnitude to overcome drag
BearingHold.STUCK_RATE = 2.0       -- deg/s; below this the hull is not really turning,
                                   -- so MIN_OUTPUT may kick it off a steady-state hang
BearingHold.DEADBAND   = 2.0       -- degrees; bearing errors within this range are ignored

function BearingHold:new(navTable)
    local t = setmetatable({}, { __index = BearingHold })
    t.navTable  = navTable
    t.integral  = ClampedIntegral:new(BearingHold.I_LIMIT)
    t.lastBearing = nil
    t.lastRate = 0
    t.lastClock = nil
    return t
end

-- Reset the integral on engagement so the controller starts from a clean state.
-- The proportional term alone handles the initial correction; seeding the
-- integral to zero avoids carrying over stale windup from a previous engagement.
function BearingHold:captureState()
    if not self.navTable.hasTarget() then return end
    self.integral:reset()
    self.lastBearing = nil  -- no stale rate across an engagement gap
    self.lastRate = 0
end

-- Returns a PID steering value in [-1, 1] based on bearing error, or 0 if no
-- target. Call once per tick; the derivative term is stateful.
function BearingHold:read()
    if not self.navTable.hasTarget() then
        self.lastBearing = nil
        self.lastRate    = 0
        self.lastOutput  = nil
        self.integral:reset()
        return 0
    end
    local bearing = self.navTable.getBearing()
    -- Wrap to [-180, 180] so the ship always turns the short way
    bearing = ((bearing + 180) % 360) - 180
    local dt = stepClock(self, 0.1)

    -- Bearing rate, wrapped so a ±180 crossing is not read as a huge slew,
    -- then low-passed so the D term reacts to the swing and not to the
    -- quantization step of a single reading.
    if self.lastBearing ~= nil then
        local delta = ((bearing - self.lastBearing + 180) % 360) - 180
        local raw = delta / dt
        self.lastRate = self.lastRate
            + BearingHold.D_SMOOTH * (raw - self.lastRate)
    end
    self.lastBearing = bearing
    local rate = self.lastRate

    -- Within deadband: reset integral and output zero
    if math.abs(bearing) <= BearingHold.DEADBAND then
        self.integral:reset()
        self.lastOutput = 0
        return 0
    end

    -- Integrate in degree-seconds, but only near the target. Outside I_BAND
    -- the proportional term already commands most of the available lock, so
    -- accumulating there just guarantees an overshoot on arrival.
    if math.abs(bearing) <= BearingHold.I_BAND then
        self.integral:add(bearing * dt)
    else
        self.integral:reset()
    end

    local value = (bearing * BearingHold.GAIN)
        + (self.integral.value * BearingHold.I_GAIN)
        + (rate * BearingHold.D_GAIN)
    value = Range:new(-1, 1):clamp(value)

    -- Minimum output floor only when the hull is not already turning: this
    -- exists to break a steady-state hang, not to force lock into a swing
    -- that is already closing on the target.
    if math.abs(rate) < BearingHold.STUCK_RATE then
        if value > 0 and value < BearingHold.MIN_OUTPUT then
            value = BearingHold.MIN_OUTPUT
        elseif value < 0 and value > -BearingHold.MIN_OUTPUT then
            value = -BearingHold.MIN_OUTPUT
        end
    end
    self.lastOutput = value
    return value
end

-- VelocityHold maintains a target velocity and computes a PI control
-- output to drive it. Its read() method is compatible as a MixerChannel input.
VelocityHold = {}

VelocityHold.GAIN    = 15.0  -- proportional gain; tune as needed
VelocityHold.I_GAIN  = 2.0   -- integral gain; tune as needed
VelocityHold.I_LIMIT = 15.0  -- clamps the integral to prevent windup

function VelocityHold:new(velocitySensor, minOutput, maxOutput)
    local t = setmetatable({}, { __index = VelocityHold })
    t.sensor   = velocitySensor
    t.target   = 0
    t.integral = ClampedIntegral:new(VelocityHold.I_LIMIT)
    t.output   = Range:new(minOutput or 0, maxOutput or VelocityHold.I_LIMIT)
    return t
end

-- Set an explicit target velocity. Deliberately does NOT reset the integral:
-- the throttle lever changes continuously, and the integral is the power the
-- controller has already found. Resetting on every notch would bump output.
function VelocityHold:setTarget(velocity)
    self.target = velocity
end

-- Nudge the target velocity by a small amount without resetting the integral.
-- Use this for incremental adjustments (e.g. button presses) to avoid sudden
-- control changes.
function VelocityHold:nudgeTarget(delta)
    self.target = self.target + delta
end

-- Park the loop: command zero power and drop the accumulated integral.
-- The propellers cannot reverse, so at the stop detent there is nothing for
-- the integral to hold; leaving it would keep pushing thrust at zero error.
function VelocityHold:holdOff()
    self.target = 0
    self.integral:reset()
    return 0
end

-- Capture the current velocity as the target (used when engaging hold mode).
-- Optionally pass the current throttle to seed the integral so output starts
-- smoothly from the current power level rather than from zero.
function VelocityHold:captureTarget(currentThrottle)
    self:setTarget(self.sensor.getVelocity())
    if currentThrottle ~= nil then
        -- Back-calculate integral so initial output matches current throttle.
        -- At capture moment error is 0, so output = integral * I_GAIN.
        self.integral:set(currentThrottle / VelocityHold.I_GAIN)
    end
end

-- Returns a PI power value based on velocity error.
-- Compatible as a MixerChannel input read function.
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
VNav.PROP_GAIN      = 5.0   -- prop power per m/s of positive rate error (3 m/s -> 15)
VNav.PROP_MAX_POWER = 15

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

-- No-integral leftover boost. Positive rateError only; negative error
-- (need more sink) never spins the vertical props.
function verticalPropPower(rateError, slewLimited, hasGround)
    if rateError == nil or rateError <= VNav.PROP_DEADBAND then
        return 0
    end
    if not (slewLimited or hasGround) then
        return 0
    end
    return Range:new(0, VNav.PROP_MAX_POWER):clamp(rateError * VNav.PROP_GAIN)
end

-- PitchHold is the attitude outer loop: gimbal pitch error -> desired
-- stabilizer angle. The Servo (flight.lua) is the only thing that talks
-- to the rotational speed controller; this just writes a setpoint.
-- invertPitch in SHIP.ATT is applied by the caller if a positive
-- stabilizer angle pitches the hull the wrong way.
PitchHold = {}

PitchHold.GAIN     = 2.0    -- stabilizer deg per deg of pitch error
PitchHold.D_GAIN   = 0.4    -- stabilizer deg per deg/s of pitch rate (damping)
PitchHold.I_GAIN   = 0.15   -- stabilizer deg per degree-second of accumulated error
PitchHold.I_LIMIT  = 20.0   -- degree-seconds; I term max is I_LIMIT * I_GAIN (3.0)
PitchHold.I_BAND   = 10.0   -- degrees; only integrate near level
PitchHold.DEADBAND = 0.5    -- degrees; pitch errors within this range command 0

-- CC peripherals may return a list or multiple values. Accept either.
local function unpackReading(first, second, third)
    if type(first) == "table" then
        return first[1], first[2], first[3]
    end
    return first, second, third
end

local function isFiniteNumber(v)
    return type(v) == "number" and v == v
end

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

-- Returns a desired stabilizer angle clamped to SHIP.ATT travel.
-- Call once per tick; the integral is stateful.
function PitchHold:read()
    if self.gimbal == nil then
        self.fault = true
        return self.lastOutput
    end

    local pitch, _roll = unpackReading(self.gimbal.getAngles())
    local wx = unpackReading(self.gimbal.getAngularRates())
    if not isFiniteNumber(pitch) then
        self.fault = true
        return self.lastOutput
    end
    self.fault = false
    self.lastPitch = pitch
    if isFiniteNumber(wx) then
        self.lastPitchRate = wx
    else
        wx = self.lastPitchRate
    end

    local dt = stepClock(self, 0.1)

    -- Target is level (0). Pitch is already the error.
    if math.abs(pitch) <= PitchHold.DEADBAND then
        self.integral:reset()
        self.lastOutput = 0
        return 0
    end

    if math.abs(pitch) <= PitchHold.I_BAND then
        self.integral:add(pitch * dt)
    else
        self.integral:reset()
    end

    local value = (pitch * PitchHold.GAIN)
        + (self.integral.value * PitchHold.I_GAIN)
        + (wx * PitchHold.D_GAIN)
    value = self.travel:clamp(value)
    self.lastOutput = value
    return value
end
