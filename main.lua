require("flight")
require("controls")
require("autopilot")
require("display")
require("util")

local propeller1 = Propeller:new("analog_transmission_6")
local propeller2 = Propeller:new("analog_transmission_7")
local throttleLever = peripheral.wrap("throttle_lever_3")
local velocitySensor = peripheral.wrap("velocity_sensor_2")
local steeringWheel = peripheral.wrap("steering_wheel_2")
local navigationTable = peripheral.wrap("navigation_table_0")

local MAX_POWER = 15
local STEERING_OFFSET = MAX_POWER / 2  -- max differential at full steering lock
local cachedSpeed = 0  -- computed once per tick to avoid double-calling velocityHold:read()
local lastToggleState = redstone.getInput("top")

local leverOutput = OnChangeOutput:new(function(v)
    throttleLever.setSignal(v)
end, 0, true)

local velocityHold = VelocityHold:new(
    velocitySensor,
    function(v) leverOutput:set(v) end,
    0, MAX_POWER
)

local display = FlightDisplay:new()

local function activeSpeed()
    return cachedSpeed
end

-- Right propeller: base speed + steering offset
local rightMixer = MixerChannel:new(
    function(v) propeller1:setPower(v) end,
    0, MAX_POWER,
    { read = activeSpeed,                                                  weight = 1.0 },
    { read = function() return (steeringWheel.getAngle() or 0) / 180 end, weight =  STEERING_OFFSET }
)

-- Left propeller: base speed - steering offset (negative weight inverts the differential)
local leftMixer = MixerChannel:new(
    function(v) propeller2:setPower(v) end,
    0, MAX_POWER,
    { read = activeSpeed,                                                  weight = 1.0 },
    { read = function() return (steeringWheel.getAngle() or 0) / 180 end, weight = -STEERING_OFFSET }
)

local function controlUpdate()
    local toggleState = redstone.getInput("top")

    -- Detect transition into hold mode and capture current velocity as target
    if toggleState and not lastToggleState then
        leverOutput.lastValue = nil
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

    display:update({
        velocity       = velocitySensor.getVelocity(),
        throttle       = throttleLever.getState(),
        holdMode       = toggleState,
        targetVelocity = velocityHold.target,
        steeringAngle  = steeringWheel.getAngle() or 0,
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
            velocityHold:nudgeTarget(0.1)
        elseif action == "dec" then
            velocityHold:nudgeTarget(-0.1)
        end
    end
end
