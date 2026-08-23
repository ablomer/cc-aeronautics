require("util")
require("config")

-- HeadingHold tracks a 0-359 heading setpoint and produces a PID steering
-- output in [-1, 1]. Same cascade shape as VNAV: an outer source (NAV
-- bearing or the wheel as a turn-rate command) writes the setpoint; this
-- inner loop closes on it. Proportional is linear across the full wrap:
-- -180° → -1, +180° → +1. The D term brakes a fast close so the hull
-- does not carry yaw through 0.
HeadingHold = {}

HeadingHold.GAIN       = 1 / 180   -- proportional; maps ±180° error to ±1.0 (19° → ~0.11)
HeadingHold.D_GAIN     = 0.009     -- output per degree/second of closing rate (damping);
                                   -- raise if it still overshoots, lower if it crawls in
HeadingHold.D_SMOOTH   = 0.4       -- rate low-pass weight; used only on the
                                   -- differentiated-heading fallback (gimbal path is clean)
HeadingHold.I_GAIN     = 0.03      -- output per degree-second; max I is
                                   -- I_LIMIT * I_GAIN (0.24) — enough to use
                                   -- maxHoldFarDiff without eating the near cap
HeadingHold.I_LIMIT    = 8.0       -- degree-seconds
HeadingHold.I_BAND     = 40.0      -- degrees; hold-trim only. I is frozen while the
                                   -- wheel is commanding a rate so the lead-clamp
                                   -- error cannot wind a leftover yaw on center
HeadingHold.FF_GAIN    = 0.30      -- fraction of full STEER at maxTurnRate; the wheel
                                   -- is a rate command, so it should produce STEER
                                   -- immediately rather than waiting for heading lag
HeadingHold.MIN_OUTPUT = 0.05      -- ~13 RPM at maxSteerDiff 256; NAV / rate-command
                                   -- hang-breaker only — not used while holding
HeadingHold.STUCK_RATE = 2.0       -- deg/s; below this the hull is not really turning,
                                   -- so MIN_OUTPUT may kick it off a steady-state hang
HeadingHold.DEADBAND   = 2.0       -- degrees; P shrinks to 0 here, I holds the trim
HeadingHold.HOLD_TAPER = 8.0       -- deg; full maxHoldDiff at/above this, then
                                   -- linear fade toward HOLD_NEAR at the deadband
HeadingHold.HOLD_NEAR  = 0.15      -- fraction of maxHoldDiff at the deadband edge
HeadingHold.HOLD_FAR   = 20.0      -- deg; grow from maxHoldDiff to maxHoldFarDiff
HeadingHold.I_BLEED    = 3.0       -- 1/s; leak I while error is clearly closing
HeadingHold.I_BLEED_HYST = 1.5     -- deg; ignore smaller |err| drops so heading
                                   -- noise at speed cannot dump the cruise trim
HeadingHold.STOP_LEAD  = 0.35      -- seconds of yaw coast credited when the wheel
                                   -- returns to center; the setpoint snaps to where
                                   -- the hull will settle instead of staying up to
                                   -- maxHeadingLead ahead of it
HeadingHold.STOP_MARGIN = 2.5      -- multiple of the D term the hold cap must allow,
                                   -- so the taper can never clamp away the authority
                                   -- needed to arrest a yaw already in progress

-- STEER ceiling while the wheel is centered. Tapers with |error| so a
-- 3° residual cannot spend the whole 48 RPM and carry through zero, but
-- never below what the D term needs to stop an existing yaw.
-- Turns (nonzero commanded rate) and NAV (nil rate) keep [-1, 1].
local function holdSteerLimit(err, rate)
    local maxDiff = SHIP.LNAV.maxSteerDiff
    if maxDiff == nil or maxDiff <= 0 then return 1 end
    local hold = SHIP.LNAV.maxHoldDiff or maxDiff
    local far = SHIP.LNAV.maxHoldFarDiff or hold
    if far < hold then far = hold end
    local full = Range:new(0, 1):clamp(hold / maxDiff)
    local farLim = Range:new(0, 1):clamp(far / maxDiff)
    if err == nil then return full end

    local base
    local ae = math.abs(err)
    local farErr = HeadingHold.HOLD_FAR
    if ae <= HeadingHold.DEADBAND then
        base = full * HeadingHold.HOLD_NEAR
    elseif ae < HeadingHold.HOLD_TAPER then
        local span = HeadingHold.HOLD_TAPER - HeadingHold.DEADBAND
        if span <= 0 then
            base = full * HeadingHold.HOLD_NEAR
        else
            local t = (ae - HeadingHold.DEADBAND) / span
            local near = full * HeadingHold.HOLD_NEAR
            base = near + t * (full - near)
        end
    elseif ae >= farErr or farErr <= HeadingHold.HOLD_TAPER then
        -- Persistent offset (cruise veer): grow toward maxHoldFarDiff.
        base = farLim
    else
        local t = (ae - HeadingHold.HOLD_TAPER) / (farErr - HeadingHold.HOLD_TAPER)
        base = full + t * (farLim - full)
    end

    -- Yaw already in progress needs braking authority regardless of how
    -- small the error is. At hover-weave rates (~2 deg/s) this adds
    -- almost nothing; after a full-lock turn it unlocks the real stop.
    local damp = math.abs((rate or 0) * HeadingHold.D_GAIN) * HeadingHold.STOP_MARGIN
    if damp > farLim then damp = farLim end
    if damp > base then return damp end
    return base
end

-- CC peripherals may return a list or multiple values. Accept either.
local function unpackReading(first, second, third)
    if type(first) == "table" then
        return first[1], first[2], first[3]
    end
    return first, second, third
end

function HeadingHold:new(navTable, gimbal)
    local t = setmetatable({}, { __index = HeadingHold })
    t.navTable  = navTable
    t.gimbal    = gimbal
    t.integral  = ClampedIntegral:new(HeadingHold.I_LIMIT)
    t.target    = 0
    t.lastHeading = nil
    t.lastRate = 0
    t.lastError = 0
    t.lastOutput = 0
    t.lastClock = nil
    t.fault = false
    t.rateCommanded = false
    return t
end

-- Seed the setpoint from the live compass heading and clear integrator /
-- rate history. Used on the first tick, on unpark from landed, and when
-- NAV drops out so the wheel holds whatever heading the hull is on.
function HeadingHold:captureHeading()
    local heading = compassHeading(self.navTable.getHeading())
    if isFiniteNumber(heading) then
        self.target = heading
        self.lastHeading = heading
        self.fault = false
    else
        self.fault = true
    end
    self.integral:reset()
    self.lastRate = 0
    self.lastClock = nil
    self.rateCommanded = false
end

function HeadingHold:setTarget(heading)
    if isFiniteNumber(heading) then
        self.target = norm360(heading)
    end
end

-- Advance the setpoint at the commanded turn rate (deg/s). Blocks growth
-- past maxHeadingLead but always permits movement that reduces the error,
-- so a pilot reversing out of a saturated turn is never locked out.
function HeadingHold:advanceTarget(rate, dt, heading)
    if rate == 0 then return end
    local err = headingError(self.target, heading)
    local proposed = norm360(self.target + rate * dt)
    local newErr = headingError(proposed, heading)
    if math.abs(newErr) > SHIP.LNAV.maxHeadingLead
        and math.abs(newErr) > math.abs(err) then
        return
    end
    self.target = proposed
end

-- d(headingError)/dt for the D term. Gimbal wy is already that sign
-- (right-hand-rule about body-up: a right turn is negative wy, which
-- matches a closing positive heading error). Compass heading rate is
-- -wy, the same negation compassHeading applies; error rate is then -(-wy).
-- Falls back to differentiated heading (smoothed) if the gimbal is down.
-- lastHeading must be assigned by the caller AFTER this returns.
function HeadingHold:yawRate(heading, dt)
    if self.gimbal ~= nil then
        local _, wy = unpackReading(self.gimbal.getAngularRates())
        if isFiniteNumber(wy) then
            return wy
        end
    end
    if self.lastHeading ~= nil then
        local headingRate = wrap180(heading - self.lastHeading) / dt
        local raw = -headingRate
        self.lastRate = self.lastRate
            + HeadingHold.D_SMOOTH * (raw - self.lastRate)
    end
    return self.lastRate
end

-- Returns a PID steering value in [-1, 1]. commandedRate is deg/s of
-- setpoint advance (wheel HDG mode), or nil when the target was set
-- externally (NAV). Call once per tick; integral / D / dt are stateful.
function HeadingHold:read(commandedRate)
    local heading = compassHeading(self.navTable.getHeading())
    local dt = stepClock(self, 0.1)

    if not isFiniteNumber(heading) then
        self.fault = true
        self.lastOutput = 0
        return 0
    end
    self.fault = false

    if commandedRate ~= nil then
        self:advanceTarget(commandedRate, dt, heading)
    end

    local rate = self:yawRate(heading, dt)
    self.lastRate = rate
    self.lastHeading = heading

    -- Wheel just came back to center. The setpoint may be up to
    -- maxHeadingLead ahead of the hull, which reads to the pilot as the
    -- ship ignoring them and turning on its own. Snap it to where the
    -- yaw will actually coast to, so centering means "stop here".
    -- Compass heading rate is -rate, hence the subtraction.
    local wheelHold = commandedRate == 0
    if wheelHold and self.rateCommanded then
        self:setTarget(heading - rate * HeadingHold.STOP_LEAD)
    end
    self.rateCommanded = commandedRate ~= nil and commandedRate ~= 0

    local err = headingError(self.target, heading)
    local prevErr = self.lastError
    self.lastError = err

    -- Same rule as PitchHold: a leftover heading is a trim problem.
    -- Zeroing I (or the output) inside the deadband dumps the offset
    -- that counters a constant yaw bias, and the hull walks right back
    -- out. Shrink P to 0 across the band so the edge is continuous;
    -- freeze I inside the band and keep integrating a residual until
    -- we get there. Reset I only on a huge intercept so a 180° swing
    -- cannot wind up.
    --
    -- I is hold-trim only. While the wheel is commanding a rate the
    -- heading error is mostly lead-clamp lag, not bias — integrating
    -- that, then freezing it on center, was a leftover yaw command
    -- that made the ship hunt.
    local shrunk = 0
    if err > HeadingHold.DEADBAND then
        shrunk = err - HeadingHold.DEADBAND
    elseif err < -HeadingHold.DEADBAND then
        shrunk = err + HeadingHold.DEADBAND
    end

    -- I learns bias only with the wheel centered. NAV (nil) still
    -- integrates on the last I_BAND of an intercept; a rate command
    -- freezes the last hold trim.
    local mayIntegrate = commandedRate == nil or wheelHold
    if math.abs(err) > HeadingHold.I_BAND then
        self.integral:reset()
    elseif mayIntegrate and math.abs(err) > HeadingHold.DEADBAND then
        -- Accumulate while the error is growing or holding. Bleed
        -- only on a clear close (past I_BLEED_HYST) so a 1° heading
        -- wobble at speed cannot dump the cruise trim.
        local closing = math.abs(err) < math.abs(prevErr) - HeadingHold.I_BLEED_HYST
        if closing then
            local keep = math.exp(-HeadingHold.I_BLEED * dt)
            self.integral:set(self.integral.value * keep)
        else
            self.integral:add(err * dt)
        end
    end

    local value = (shrunk * HeadingHold.GAIN)
        + (self.integral.value * HeadingHold.I_GAIN)
        + (rate * HeadingHold.D_GAIN)
    if commandedRate ~= nil and SHIP.LNAV.maxTurnRate > 0 then
        value = value + (commandedRate / SHIP.LNAV.maxTurnRate) * HeadingHold.FF_GAIN
    end
    if wheelHold then
        local lim = holdSteerLimit(err, rate)
        value = Range:new(-lim, lim):clamp(value)
    else
        value = Range:new(-1, 1):clamp(value)
    end

    -- MIN_OUTPUT is a hang-breaker for NAV / rate commands. On hold it
    -- is the punch that carried a 3° residual through zero by ~5°.
    if not wheelHold
        and math.abs(err) > HeadingHold.DEADBAND
        and math.abs(rate) < HeadingHold.STUCK_RATE then
        local floor = HeadingHold.MIN_OUTPUT
        if value > 0 and value < floor then
            value = floor
        elseif value < 0 and value > -floor then
            value = -floor
        end
    end
    self.lastOutput = value
    return value
end

-- Park the loop: zero steer, reset the integral, and recapture the target
-- from the live heading so a later unpark is bumpless.
function HeadingHold:holdOff()
    self.integral:reset()
    self.lastRate = 0
    self.lastError = 0
    self.lastOutput = 0
    self.lastClock = nil
    self.rateCommanded = false
    local heading = compassHeading(self.navTable.getHeading())
    if isFiniteNumber(heading) then
        self.target = heading
        self.lastHeading = heading
        self.fault = false
    else
        self.fault = true
    end
    return 0
end

-- VelocityHold maintains a target velocity and computes a PI control
-- output in propeller RPM (the surge input to allocatePropMix).
VelocityHold = {}

-- Analog loop was 15 power per 1 m/s of error (full scale). Same fraction
-- of available RSC RPM: 1 m/s commands SHIP.LNAV.maxRpm.
VelocityHold.GAIN    = SHIP.LNAV.maxRpm
VelocityHold.I_GAIN  = 2.0 * (SHIP.LNAV.maxRpm / 15)  -- same I/P ratio as analog
VelocityHold.I_LIMIT = 15.0  -- accumulated velocity-error ticks; not RPM

function VelocityHold:new(velocitySensor, minOutput, maxOutput)
    local t = setmetatable({}, { __index = VelocityHold })
    t.sensor   = velocitySensor
    t.target   = 0
    t.integral = ClampedIntegral:new(VelocityHold.I_LIMIT)
    t.maxOutput = maxOutput or SHIP.LNAV.maxRpm
    t.output   = Range:new(minOutput or 0, t.maxOutput)
    return t
end

-- Lower this tick's output ceiling to the RPM the mixer can actually
-- deliver after the yaw differential takes its share. Anti-windup keys
-- off output.max, so without this the loop keeps integrating against a
-- limit the props never reach during a sustained turn and then slams
-- surge the moment the turn ends. Call every tick: it restores itself
-- to maxOutput as the differential shrinks.
function VelocityHold:setCeiling(rpm)
    self.output.max = Range:new(self.output.min, self.maxOutput):clamp(rpm)
end

-- Set an explicit target velocity. Deliberately does NOT reset the integral:
-- the throttle lever changes continuously, and the integral is the RPM the
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

-- Park the loop: drop the speed target and integral. Detent 0 is not a
-- 0 m/s hold; pivot steering still uses the wheel at this detent.
function VelocityHold:holdOff()
    self.target = 0
    self.integral:reset()
    return 0
end

-- Capture the current velocity as the target (used when engaging hold mode).
-- Optionally pass the current RPM to seed the integral so output starts
-- smoothly from the current command rather than from zero.
function VelocityHold:captureTarget(currentRpm)
    self:setTarget(self.sensor.getVelocity())
    if currentRpm ~= nil then
        -- Back-calculate integral so initial output matches current RPM.
        -- At capture moment error is 0, so output = integral * I_GAIN.
        self.integral:set(currentRpm / VelocityHold.I_GAIN)
    end
end

-- Returns a PI RPM value based on velocity error, clamped to the current
-- output range (see setCeiling).
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
