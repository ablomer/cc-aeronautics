require("flight")
require("controls")
require("autopilot")
require("display")

local propeller = Propeller:new("analog_transmission_0")
local throttleLever = peripheral.wrap("throttle_lever_0")
local velocitySensor = peripheral.wrap("velocity_sensor_0")

local leverOutput = OnChangeOutput:new(function(v) throttleLever.setSignal(v) end)
local velocityHold = VelocityHold:new(
    velocitySensor,
    function(v) leverOutput:set(v) end
)
local display = FlightDisplay:new()

local manualChannel = ControlChannel:new(
    function() return throttleLever.getState() end,
    function(v) propeller:setPower(v) end
    -- To drive two propellers with one throttle, add a second binding:
    -- function(v) secondPropeller:setPower(v) end
)

local holdChannel = ControlChannel:new(
    function() return velocityHold:read() end,
    function(v) propeller:setPower(v) end
)

local lastToggleState = redstone.getInput("left")

while true do
    local toggleState = redstone.getInput("left")

    -- Detect transition into hold mode and capture current velocity as target
    if toggleState and not lastToggleState then
        leverOutput.lastValue = nil
        velocityHold:captureTarget()
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

    sleep(0.2)
end
