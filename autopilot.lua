require("util")

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

function VelocityHold:new(velocitySensor, onOutput, minOutput, maxOutput)
    local t = setmetatable({}, { __index = VelocityHold })
    t.sensor   = velocitySensor
    t.target   = 0
    t.integral = ClampedIntegral:new(VelocityHold.I_LIMIT)
    t.onOutput = onOutput    -- optional callback(value) fired when a power value is computed
    t.output   = Range:new(minOutput or 0, maxOutput or VelocityHold.I_LIMIT)
    return t
end

-- Set an explicit target velocity (e.g. from a number input: hold:setTarget(10))
function VelocityHold:setTarget(velocity)
    self.target = velocity
    self.integral:reset()  -- reset integral on target change to avoid windup carry-over
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
    if self.onOutput then self.onOutput(value) end
    return value
end

-- AltitudeHold maintains a target altitude using a cascade of two loops
-- instead of a single PID against a calibration curve:
--   outer loop:  altitude error      -> desired vertical speed (clamped, asymmetric)
--   inner loop:  vertical speed error -> burner amount (PI)
-- Because each burner amount deterministically settles at one altitude, the
-- inner integral naturally converges on the correct equilibrium amount for
-- the current target: vertical speed can only be zero at that one amount.
-- No calibration curve or pre-flight tuning flight is required.
--
-- NOT verified against the physical peripherals in this workspace: the sign
-- convention of getVerticalSpeed() (positive = climbing is assumed) and the
-- exact climb/descend authority of the burners. Confirm in-game before
-- trusting the default gains below, they are starting points only.
AltitudeHold = {}

-- Operational altitude range. MAX_ALTITUDE is deliberately below the true
-- sensor/world ceiling (320) to leave braking margin; HARD_CEILING is a last-
-- resort override independent of the control loop.
AltitudeHold.MIN_ALTITUDE  = 60
AltitudeHold.MAX_ALTITUDE  = 315
AltitudeHold.HARD_CEILING  = 318   -- above this, force minimum burner amount regardless of target/output

-- Burner amount range comes from the shared BURNER_AMOUNT_RANGE (util.lua) so
-- this controller's clamping/anti-windup always agrees with what BurnerBank
-- (flight.lua) actually sends to the peripherals.

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
-- This only changes the outer loop's rate *setpoint*; it does not touch the
-- inner loop's gains, integral, or slew logic, so vertical-speed tracking
-- behavior is unaffected.
AltitudeHold.DEADBAND = 3.0   -- metres, tune in-game

-- Inner loop: PI on vertical-speed error, producing a burner amount.
AltitudeHold.RATE_P_GAIN  = 40.0
AltitudeHold.RATE_I_GAIN  = 10.0
AltitudeHold.RATE_I_LIMIT = 200.0

-- Slew limit: burner amount may increase at most this fast (per second) to
-- avoid abrupt heat spikes that overshoot; decreases are never slew-limited
-- since cutting heat is the safe direction.
AltitudeHold.MAX_INCREASE_RATE = 150.0

local function isValidHeight(h)
    return type(h) == "number" and h == h and h > -1000 and h < 1000
end

local function isValidRate(v)
    return type(v) == "number" and v == v
end

function AltitudeHold:new(altitudeSensor)
    local t = setmetatable({}, { __index = AltitudeHold })
    t.sensor    = altitudeSensor
    t.target    = AltitudeHold.MIN_ALTITUDE
    t.integral  = ClampedIntegral:new(AltitudeHold.RATE_I_LIMIT)
    t.lastAmount = BURNER_AMOUNT_RANGE.min
    t.lastClock  = nil
    t.fault      = false
    t.lastHeight = nil
    t.lastVerticalSpeed = nil
    return t
end

-- Set an explicit target altitude, clamped to the operational range.
-- Deliberately does NOT reset the integral: the target changes continuously
-- as the lever moves, and the integral represents the burner amount the
-- controller has already found for the current regime. Resetting it on
-- every lever nudge would discard that and reintroduce bump/overshoot.
function AltitudeHold:setTarget(altitude)
    self.target = Range:new(AltitudeHold.MIN_ALTITUDE, AltitudeHold.MAX_ALTITUDE):clamp(altitude)
end

-- Seed the integral from a known current burner amount for bumpless startup
-- (e.g. when the control loop first engages). At the capture moment we don't
-- know the actual rate error, so this assumes it is near zero.
function AltitudeHold:captureTarget(currentAmount)
    if currentAmount ~= nil then
        self.integral:set(currentAmount / AltitudeHold.RATE_I_GAIN)
    end
    self.lastClock = nil  -- force dt recalibration on next read
end

-- Returns a burner amount clamped to BURNER_AMOUNT_RANGE. Call once per tick;
-- this is stateful (integral, slew, dt) like VelocityHold:read().
function AltitudeHold:read()
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
        -- integrating on bad data or guessing a new one.
        self.fault = true
        return self.lastAmount
    end
    self.fault = false

    -- Hard override: independent of target/controller, never allow the
    -- computed output to push past the ceiling margin.
    if height >= AltitudeHold.HARD_CEILING then
        self.integral:set(math.min(self.integral.value, 0))
        self.lastAmount = BURNER_AMOUNT_RANGE.min
        return self.lastAmount
    end

    -- Outer loop: altitude error -> desired vertical speed.
    -- Shrink the error by the deadband (rather than hard-zeroing inside it)
    -- so desiredRate is continuous across the band edge and still shrinks
    -- smoothly to zero as height approaches the target.
    local altError = self.target - height
    local shrunkError = 0
    if altError > AltitudeHold.DEADBAND then
        shrunkError = altError - AltitudeHold.DEADBAND
    elseif altError < -AltitudeHold.DEADBAND then
        shrunkError = altError + AltitudeHold.DEADBAND
    end
    local desiredRate = shrunkError * AltitudeHold.APPROACH_GAIN
    desiredRate = Range:new(-AltitudeHold.MAX_DESCENT_RATE, AltitudeHold.MAX_CLIMB_RATE):clamp(desiredRate)

    -- Inner loop: vertical speed error -> burner amount (PI)
    local rateError = desiredRate - verticalSpeed
    local proposed = (rateError * AltitudeHold.RATE_P_GAIN)
        + ((self.integral.value + rateError * dt) * AltitudeHold.RATE_I_GAIN)

    -- Conditional anti-windup: only accumulate if doing so wouldn't push the
    -- output further past a limit it has already saturated against.
    local willSaturateHigh = proposed > BURNER_AMOUNT_RANGE.max and rateError > 0
    local willSaturateLow  = proposed < BURNER_AMOUNT_RANGE.min and rateError < 0
    if not (willSaturateHigh or willSaturateLow) then
        self.integral:add(rateError * dt)
    end

    local amount = (rateError * AltitudeHold.RATE_P_GAIN) + (self.integral.value * AltitudeHold.RATE_I_GAIN)
    amount = BURNER_AMOUNT_RANGE:clamp(amount)

    -- Slew limit increases only; decreases apply immediately for safety.
    if amount > self.lastAmount then
        amount = math.min(amount, self.lastAmount + AltitudeHold.MAX_INCREASE_RATE * dt)
    end

    self.lastAmount = amount
    self.lastHeight = height
    self.lastVerticalSpeed = verticalSpeed
    self.lastDesiredRate = desiredRate
    return amount
end
