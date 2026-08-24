require("util")
require("config")

-- Propeller drives a Create rotational speed controller. Unlike the old
-- analog transmissions (brakes: higher signal = slower), setTargetSpeed
-- commands RPM directly. Range is integer [-256, 256]; the peripheral
-- clamps anything outside that.
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

function Propeller:setSpeed(speed)
    speed = roundRpm(speed)
    speed = Range:new(-self.maxRpm, self.maxRpm):clamp(speed)
    self.lastSpeed = speed
    local commanded = self.invert and -speed or speed
    if self.lastCommanded == nil or commanded ~= self.lastCommanded then
        if self.rsc ~= nil then
            self.rsc.setTargetSpeed(commanded)
        end
        self.lastCommanded = commanded
    end
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

function DifferentialThrustMixer:apply(speedReq, steerReq)
    local result = mixDifferentialThrust(speedReq, steerReq, self.opts)
    self.left:setSpeed(result.leftRpm)
    self.right:setSpeed(result.rightRpm)
    result.leftRpm = self.left.lastSpeed
    result.rightRpm = self.right.lastSpeed
    self.last = result
    return result
end

-- Analog transmissions still used as brakes on the vertical prop bank.
-- Higher setSignal slows the shaft; setPower inverts so 15 is full speed.
AnalogPropeller = {}

function AnalogPropeller:new(transmission)
    local t = setmetatable({}, { __index = AnalogPropeller })
    t.transmission = peripheral.wrap(transmission)
    t.lastPower = 0
    return t
end

function AnalogPropeller:setPower(power)
    self.transmission.setSignal(15 - power)
    self.lastPower = power
end

-- BurnerBank fans one commanded amount out to every hot air burner in the
-- collection. Clamping/rounding happens here at the actuator boundary so
-- callers (controllers) never have to worry about the peripheral's accepted
-- range.
BurnerBank = {}

function BurnerBank:new(burners)
    local t = setmetatable({}, { __index = BurnerBank })
    t.burners = burners
    t.lastAmount = BURNER_AMOUNT_RANGE.min
    return t
end

function BurnerBank:setAmount(amount)
    amount = BURNER_AMOUNT_RANGE:clamp(amount)
    for _, burner in ipairs(self.burners) do
        burner.setTargetAmount(amount)
    end
    self.lastAmount = amount
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

function Servo:stop()
    if self.rsc ~= nil then
        self.rsc.setTargetSpeed(0)
    end
    self.lastSpeed = 0
end

function Servo:update()
    if not self.available then
        return 0
    end

    local angle = self.bearing.getAngle()
    if not isFiniteNumber(angle) then
        self.fault = true
        self.lastError = 0
        self:stop()
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
        self.rsc.setTargetSpeed(speed)
        self.lastSpeed = speed
    end
    return speed
end
