require("flight")

local propeller1 = Propeller:new("analog_transmission_6")
local propeller2 = Propeller:new("analog_transmission_7")
local throttleLever = peripheral.wrap("throttle_lever_2")

print("Debug: direct setPower test. Move the throttle lever.")
print("Press Q to quit.")

while true do
    local power = throttleLever.getState()
    propeller1:setPower(power)
    propeller2:setPower(power)
    print(string.format("throttle=%d  signal=%d", power, 15 - power))
    sleep(0.2)
end
