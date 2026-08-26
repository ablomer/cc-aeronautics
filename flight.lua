require("util")
require("config")
require("controls")

-- Propeller drives a Create rotational speed controller.
-- setTargetSpeed commands RPM directly. Range is integer [-256, 256];
-- the peripheral clamps anything outside that.
-- https://wiki.createmod.net/users/cc-tweaked-integration/rotational-speed-controller#setTargetSpeed
Propeller = {}
Propeller.MAX_RPM = 256

local function roundRpm(speed)
    if speed >= 0 then
        return math.floor(speed + 0.5)
    end
    return math.ceil(speed - 0.5)
end

function Propeller:new(speedControllerId, opts)
    local t = setmetatable({}, { __index = Propeller })
    t.rsc = peripheral.wrap(speedControllerId)
    opts = opts or {}
    t.maxRpm = opts.maxRpm or Propeller.MAX_RPM
    t.invert = opts.invert or false
    t.lastSpeed = 0
    t.lastCommanded = nil
    return t
end

function Propeller:setSpeed(speed, batch)
    speed = roundRpm(speed)
    speed = Range:new(-self.maxRpm, self.maxRpm):clamp(speed)
    self.lastSpeed = speed
    local commanded = self.invert and -speed or speed
    if self.lastCommanded ~= nil and commanded == self.lastCommanded then
        return
    end
    self.lastCommanded = commanded
    if self.rsc == nil then
        return
    end
    local rsc = self.rsc
    WriteBatch.defer(batch, function()
        rsc.setTargetSpeed(commanded)
    end)
end

-- Mix a normalized speed request [0, 1] and steering request [-1, 1]
-- into left/right propeller RPM. Steering is preserved; common-mode
-- (forward) thrust is reduced when the pair would exceed maxRpm.
--
-- Sign: +steering is a right turn (left faster / right slower).
-- invertSteer flips that if the hull yaws the wrong way.
function mixDifferentialThrust(speedReq, steerReq, opts)
    opts = opts or SHIP.LNAV
    local forwardRpm = opts.forwardRpm
    local turnRpm = opts.turnRpm
    local maxRpm = opts.maxRpm
    local invertSteer = opts.invertSteer

    speedReq = Range:new(0, 1):clamp(speedReq or 0)
    local requestedSteer = Range:new(-1, 1):clamp(steerReq or 0)
    local steer = requestedSteer
    if invertSteer then
        steer = -steer
    end

    local requestedCommon = speedReq * forwardRpm
    local differential = steer * turnRpm
    local absDiff = math.abs(differential)
    if absDiff > maxRpm then
        differential = differential > 0 and maxRpm or -maxRpm
        absDiff = maxRpm
    end

    local maxCommon = maxRpm - absDiff
    if maxCommon < 0 then
        maxCommon = 0
    end
    local appliedCommon = requestedCommon
    if appliedCommon > maxCommon then
        appliedCommon = maxCommon
    end

    local leftRpm = appliedCommon + differential
    local rightRpm = appliedCommon - differential
    local limit = Range:new(-maxRpm, maxRpm)
    leftRpm = limit:clamp(leftRpm)
    rightRpm = limit:clamp(rightRpm)

    return {
        requestedSpeed = speedReq,
        requestedSteer = requestedSteer,
        requestedCommonRpm = requestedCommon,
        appliedCommonRpm = appliedCommon,
        differentialRpm = differential,
        leftRpm = leftRpm,
        rightRpm = rightRpm,
        speedReduced = appliedCommon + 0.0001 < requestedCommon,
    }
end

-- Applies mixDifferentialThrust to a left/right Propeller pair and
-- keeps the last mix result for the display snapshot.
DifferentialThrustMixer = {}

function DifferentialThrustMixer:new(leftProp, rightProp, opts)
    local t = setmetatable({}, { __index = DifferentialThrustMixer })
    t.left = leftProp
    t.right = rightProp
    t.opts = opts or SHIP.LNAV
    t.last = nil
    return t
end

function DifferentialThrustMixer:apply(speedReq, steerReq, batch)
    local result = mixDifferentialThrust(speedReq, steerReq, self.opts)
    self.left:setSpeed(result.leftRpm, batch)
    self.right:setSpeed(result.rightRpm, batch)
    result.leftRpm = self.left.lastSpeed
    result.rightRpm = self.right.lastSpeed
    self.last = result
    return result
end

-- BurnerBank fans a total heated-volume command out across every hot air
-- burner. Each burner gets floor(total / n); the last burner gets the
-- remainder so the bank can step the envelope by 1 m³. Per-burner values
-- are clamped to BURNER_AMOUNT_RANGE. Identical per-burner repeats are
-- skipped: setTargetAmount yields a server tick per burner.
BurnerBank = {}

-- Greater of the hardware per-burner floor and SHIP.VNAV.minHeatedVolume.
local function configuredMinTotal(n)
    local minVolume = n * BURNER_AMOUNT_RANGE.min
    local configured = SHIP.VNAV.minHeatedVolume
    if type(configured) == "number" and configured > minVolume then
        minVolume = configured
    end
    return minVolume
end

function BurnerBank:new(burners)
    local t = setmetatable({}, { __index = BurnerBank })
    t.burners = burners
    t.lastAmount = configuredMinTotal(#burners)
    t.lastCommanded = {}  -- per-burner; missing keys force a write
    return t
end

-- Envelope capacity from one burner (they share the balloon). 0 if none.
function BurnerBank:balloonCapacity()
    local burner = self.burners[1]
    if burner == nil or burner.getBalloonCapacity == nil then
        return 0
    end
    local cap = burner.getBalloonCapacity()
    if type(cap) ~= "number" or cap ~= cap or cap < 0 then
        return 0
    end
    return cap
end

-- Inclusive total-volume limits for this bank. Capacity 0 (no balloon
-- reading) falls back to the sum of per-burner maxima. The floor is the
-- greater of the hardware per-burner minimum and SHIP.VNAV.minHeatedVolume
-- so the hull stays upright on the ground.
function BurnerBank:volumeLimits(capacity)
    local n = #self.burners
    if n < 1 then
        return 0, 0
    end
    local minVolume = configuredMinTotal(n)
    local maxVolume = n * BURNER_AMOUNT_RANGE.max
    if type(capacity) == "number" and capacity > 0 then
        maxVolume = math.min(maxVolume, capacity)
    end
    if maxVolume < minVolume then
        maxVolume = minVolume
    end
    return minVolume, maxVolume
end

function BurnerBank:sumTargetAmounts()
    local total = 0
    for _, burner in ipairs(self.burners) do
        local amt = burner.getTargetAmount()
        if type(amt) == "number" and amt == amt then
            total = total + amt
        end
    end
    return total
end

local function splitVolume(total, n)
    total = math.floor(total + 0.5)
    local minTotal = configuredMinTotal(n)
    local maxTotal = n * BURNER_AMOUNT_RANGE.max
    if total < minTotal then
        total = minTotal
    elseif total > maxTotal then
        total = maxTotal
    end
    local base = math.floor(total / n)
    local remainder = total - base * n
    local amounts = {}
    for i = 1, n do
        local amt = base
        if i == n then
            amt = base + remainder
        end
        amounts[i] = BURNER_AMOUNT_RANGE:clamp(amt)
    end
    return amounts
end

function BurnerBank:setTotal(total, batch)
    local n = #self.burners
    if n < 1 then
        self.lastAmount = 0
        return
    end
    local amounts = splitVolume(total, n)
    local commanded = 0
    for i = 1, n do
        commanded = commanded + amounts[i]
    end
    self.lastAmount = commanded
    for i, burner in ipairs(self.burners) do
        local amt = amounts[i]
        if self.lastCommanded[i] ~= amt then
            self.lastCommanded[i] = amt
            WriteBatch.defer(batch, function()
                burner.setTargetAmount(amt)
            end)
        end
    end
end

-- Servo closes a mechanical bearing onto a target angle by commanding
-- integer RPM on a rotational speed controller. The speed law is
-- "close APPROACH of remaining error this tick", floored so a tick can
-- never travel further than the error. 1 RPM is already 6 deg/s at a
-- 1:1 bearing, so DEADBAND sits at that quantization floor.
Servo = {}
Servo.DEADBAND = 1.0   -- deg; below the 1 RPM quantization floor there is no finer control
Servo.APPROACH = 0.5   -- fraction of remaining error to close in one tick
Servo.MIN_RPM  = 1     -- integer floor of setTargetSpeed

function Servo:new(speedControllerId, bearingId, opts)
    local t = setmetatable({}, { __index = Servo })
    t.rsc = wrapOptional(speedControllerId)
    t.bearing = wrapOptional(bearingId)
    -- Both the RSC and the bearing are required to close the loop.
    -- Either missing means the stabilizer is not installed: available
    -- stays false and update() is a no-op, not a fault.
    t.available = t.rsc ~= nil and t.bearing ~= nil
    opts = opts or {}
    t.minAngle = opts.minAngle or -45
    t.maxAngle = opts.maxAngle or 45
    t.travel = Range:new(t.minAngle, t.maxAngle)
    t.maxRpm = opts.maxRpm or 8
    t.degPerSecPerRpm = opts.degPerSecPerRpm or 6.0
    t.invertServo = opts.invertServo or false
    t.target = 0
    t.lastSpeed = nil
    t.lastAngle = nil
    t.lastError = 0
    t.fault = false
    t.lastClock = nil
    -- RSC present without a bearing: park the shaft so a leftover
    -- speed command cannot keep spinning with no feedback.
    if t.rsc ~= nil and not t.available then
        t:stop()
    end
    return t
end

function Servo:setTarget(angle)
    self.target = self.travel:clamp(angle)
end

function Servo:stop(batch)
    self.lastSpeed = 0
    if self.rsc == nil then
        return
    end
    local rsc = self.rsc
    WriteBatch.defer(batch, function()
        rsc.setTargetSpeed(0)
    end)
end

function Servo:update(batch)
    if not self.available then
        return 0
    end

    local angle = self.bearing.getAngle()
    if not isFiniteNumber(angle) then
        self.fault = true
        self.lastError = 0
        self:stop(batch)
        return 0
    end
    self.fault = false
    self.lastAngle = angle

    local dt = stepClock(self, 0.1)
    local err = self.target - angle
    self.lastError = err

    local speed = 0
    if math.abs(err) > Servo.DEADBAND then
        local degPerSec = math.abs(err) * Servo.APPROACH / dt
        local rpm = degPerSec / self.degPerSecPerRpm
        rpm = math.min(self.maxRpm, rpm)
        rpm = math.floor(rpm)
        if rpm < Servo.MIN_RPM then
            rpm = Servo.MIN_RPM
        end
        if err < 0 then
            rpm = -rpm
        end
        if self.invertServo then
            rpm = -rpm
        end
        speed = rpm
    end

    if self.lastSpeed == nil or speed ~= self.lastSpeed then
        local rsc = self.rsc
        WriteBatch.defer(batch, function()
            rsc.setTargetSpeed(speed)
        end)
        self.lastSpeed = speed
    end
    return speed
end
