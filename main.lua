require("flight")
require("controls")
require("autopilot")
require("display")
require("util")

local propeller1 = Propeller:new("analog_transmission_6")
local propeller2 = Propeller:new("analog_transmission_7")
local throttleLever = peripheral.wrap("throttle_lever_2")
local velocitySensor = peripheral.wrap("velocity_sensor_2")

local leverOutput = OnChangeOutput:new(function(v) throttleLever.setSignal(v) end, 1.0)
local velocityHold = VelocityHold:new(
    velocitySensor,
    function(v) leverOutput:set(v) end
)
local display = FlightDisplay:new()

local manualChannel = ControlChannel:new(
    function() return throttleLever.getState() end,
    function(v) propeller1:setPower(v) end,
    function(v) propeller2:setPower(v) end
)

local holdChannel = ControlChannel:new(
    function() return velocityHold:read() end,
    function(v) propeller1:setPower(v) end,
    function(v) propeller2:setPower(v) end
)

local lastToggleState = redstone.getInput("top")

local function controlUpdate()
    local toggleState = redstone.getInput("top")

    -- Detect transition into hold mode and capture current velocity as target
    if toggleState and not lastToggleState then
        leverOutput.lastValue = nil
        velocityHold:captureTarget(throttleLever.getState())
    end
    lastToggleState = toggleState

    if toggleState then
        holdChannel:update()
    else
        manualChannel:update()
    end

    display:update({
        velocity       = velocitySensor.getVelocity(),
        throttle       = throttleLever.getState(),
        holdMode       = toggleState,
        targetVelocity = velocityHold.target,
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
