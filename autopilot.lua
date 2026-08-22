require("util")
require("config")

-- BearingHold steers the ship towards a navigation target by computing the
-- bearing error and producing a PI steering output in [-1, 1].
-- Its read() method is compatible as a MixerChannel input.
BearingHold = {}

BearingHold.GAIN       = 2 / 180   -- proportional gain; maps ±90° error to ±1.0 output
BearingHold.I_GAIN     = 2 / 180   -- integral gain; tune as needed
BearingHold.I_LIMIT    = 1.0       -- clamps the integral to prevent windup
BearingHold.MIN_OUTPUT = 0.1       -- minimum effective output magnitude to overcome drag
BearingHold.DEADBAND   = 2.0       -- degrees; bearing errors within this range are ignored

function BearingHold:new(navTable)
    local t = setmetatable({}, { __index = BearingHold })
    t.navTable  = navTable
    t.integral  = ClampedIntegral:new(BearingHold.I_LIMIT)
    t.lastBearing = nil
    return t
end

-- Reset the integral on engagement so the controller starts from a clean state.
-- The proportional term alone handles the initial correction; seeding the
-- integral to zero avoids carrying over stale windup from a previous engagement.
function BearingHold:captureState()
    if not self.navTable.hasTarget() then return end
    self.integral:reset()
end

-- Returns a PI steering value in [-1, 1] based on bearing error, or 0 if no target.
-- Compatible as a MixerChannel input read function.
function BearingHold:read()
    if not self.navTable.hasTarget() then
        self.lastBearing = nil
        self.lastOutput  = nil
        self.integral:reset()
        return 0
    end
    local bearing = self.navTable.getBearing()
    -- Wrap to [-180, 180] so the ship always turns the short way
    bearing = ((bearing + 180) % 360) - 180
    self.lastBearing = bearing
    -- Within deadband: reset integral and output zero
    if math.abs(bearing) <= BearingHold.DEADBAND then
        self.integral:reset()
        self.lastOutput = 0
        return 0
    end
    self.integral:add(bearing)
    local value = (bearing * BearingHold.GAIN) + (self.integral.value * BearingHold.I_GAIN)
    value = Range:new(-1, 1):clamp(value)
    -- Apply minimum output floor to overcome drag at small errors
    if value > 0 and value < BearingHold.MIN_OUTPUT then
        value = BearingHold.MIN_OUTPUT
    elseif value < 0 and value > -BearingHold.MIN_OUTPUT then
        value = -BearingHold.MIN_OUTPUT
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
    self.integral:add(error)
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

    local now = os.clock()
    local dt = 0.2
    if self.lastClock ~= nil then
        dt = now - self.lastClock
        if dt <= 0 then dt = 0.2 end
        if dt > 1.0 then dt = 1.0 end  -- guard against long stalls skewing the integral
    end
    self.lastClock = now

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
-- hull-on-ground height. Used to latch landed and cut heat.
function isTouchdown(agl, hasGround)
    return hasGround and agl <= touchdownAgl()
end

-- Lever 0: fast sink until ground contact, then linear flare from
-- LANDING_SINK at first contact (~OPTICAL_RANGE) to 0 m/s at touchdown.
function landingDesiredVS(agl, hasGround)
    if not hasGround then
        return VNav.LANDING_APPROACH_SINK
    end
    local settle = touchdownAgl()
    local span = VNav.OPTICAL_RANGE - settle
    local t = (agl - settle) / span
    t = Range:new(0, 1):clamp(t)
    return VNav.LANDING_SINK * t
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
