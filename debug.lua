-- Standalone debug menu. Run this directly (NOT through main.lua) to pick a
-- debug program from a list and run it. Press Ctrl+T to stop whichever
-- program is running and return to the shell.

require("config")
require("flight")

-- Format getter returns so tables print as {1.2, 3.4} instead of table: 0x...
local function formatValue(value)
    if type(value) == "table" then
        local parts = {}
        local n = #value
        if n > 0 then
            for i = 1, n do
                parts[i] = formatValue(value[i])
            end
            return "{" .. table.concat(parts, ", ") .. "}"
        end
        for k, v in pairs(value) do
            table.insert(parts, tostring(k) .. "=" .. formatValue(v))
        end
        return "{" .. table.concat(parts, ", ") .. "}"
    elseif type(value) == "number" then
        return string.format("%.4g", value)
    else
        return tostring(value)
    end
end

-- Several peripherals' exact method sets aren't confirmed elsewhere in this
-- workspace, so rather than hardcoding field names that might be wrong, this
-- introspects the peripheral via peripheral.getMethods and live-prints every
-- zero-argument getter it finds ("get*"/"is*"/"has*"). That keeps it useful
-- even if the real API differs from assumptions.
local function runPeripheralDebug(peripheralName, label)
    local sensor = peripheral.wrap(peripheralName)
    if not sensor then
        error("Could not find peripheral: " .. peripheralName)
    end

    local methodNames = peripheral.getMethods(peripheralName)
    table.sort(methodNames)

    -- Only call methods that look like no-argument getters. Calling arbitrary
    -- methods (e.g. setters/actions) could have side effects, so anything not
    -- named get*/is*/has* is listed but never invoked.
    local function isSafeGetter(name)
        return name:match("^get") or name:match("^is") or name:match("^has")
    end

    local getters = {}
    local others = {}
    for _, name in ipairs(methodNames) do
        if isSafeGetter(name) then
            table.insert(getters, name)
        else
            table.insert(others, name)
        end
    end

    term.clear()
    term.setCursorPos(1, 1)
    print(label .. " debug: " .. peripheralName)
    print("Press Ctrl+T to stop.")
    if #others > 0 then
        print("(not called, non-getter methods: " .. table.concat(others, ", ") .. ")")
    end

    local HEADER_LINES = 4
    local _, h = term.getSize()

    while true do
        term.setCursorPos(1, HEADER_LINES)
        for y = HEADER_LINES, h do
            term.setCursorPos(1, y)
            term.clearLine()
        end
        term.setCursorPos(1, HEADER_LINES)

        for _, name in ipairs(getters) do
            local ok, a, b, c = pcall(sensor[name])
            if ok then
                local shown
                if b ~= nil then
                    local packed = { a, b }
                    if c ~= nil then packed[3] = c end
                    shown = formatValue(packed)
                else
                    shown = formatValue(a)
                end
                print(string.format("%-20s %s", name, shown))
            else
                print(string.format("%-20s <error: %s>", name, tostring(a)))
            end
        end

        sleep(0.2)
    end
end

local function runStabilizerServoDebug()
    local servo = Servo:new(
        PERIPHERALS.ATT.stabilizerSpeedController,
        PERIPHERALS.ATT.stabilizerBearing,
        SHIP.ATT
    )
    if servo.rsc == nil or servo.bearing == nil then
        error("Could not wrap stabilizer RSC or bearing")
    end

    local lastAngle = nil
    local lastClock = nil
    local measuredRate = 0
    local NUDGE = 1.0

    local function redraw()
        term.clear()
        term.setCursorPos(1, 1)
        print("Stabilizer servo debug")
        print("Up/Down: nudge 1 deg   0: zero   Q: quit")
        print("Ctrl+T also stops the RSC before returning.")
        print()
        print(string.format("Target     %.1f deg", servo.target))
        if servo.lastAngle ~= nil then
            print(string.format("Angle      %.1f deg", servo.lastAngle))
        else
            print("Angle      --")
        end
        print(string.format("Error      %.1f deg", servo.lastError or 0))
        print(string.format("RPM        %d", servo.lastSpeed or 0))
        print(string.format("Meas. rate %.1f deg/s", measuredRate))
        print(string.format("deg/s/RPM  %.2f  invertServo=%s",
            SHIP.ATT.degPerSecPerRpm, tostring(SHIP.ATT.invertServo)))
        if servo.fault then
            print()
            print("FAULT: bearing getAngle() was not a number")
        end
    end

    local function sampleRate()
        local angle = servo.lastAngle
        local now = os.clock()
        if lastAngle ~= nil and lastClock ~= nil and angle ~= nil then
            local dt = now - lastClock
            if dt > 0 then
                measuredRate = (angle - lastAngle) / dt
            end
        end
        lastAngle = angle
        lastClock = now
    end

    redraw()
    local timer = os.startTimer(0.1)
    while true do
        -- pullEventRaw so Ctrl+T is a "terminate" event we can catch
        -- and stop the RSC instead of leaving it spinning.
        local event, p1 = os.pullEventRaw()
        if event == "terminate" then
            servo:stop()
            return
        elseif event == "timer" and p1 == timer then
            servo:update()
            sampleRate()
            redraw()
            timer = os.startTimer(0.1)
        elseif event == "key" then
            if p1 == keys.up then
                servo:setTarget(servo.target + NUDGE)
            elseif p1 == keys.down then
                servo:setTarget(servo.target - NUDGE)
            elseif p1 == keys.zero then
                servo:setTarget(0)
            elseif p1 == keys.q then
                servo:stop()
                return
            end
            redraw()
        end
    end
end

-- Add new debug programs here as { name = "...", run = function ... end }.
local PROGRAMS = {
    { name = "Laser sensor", run = function() runPeripheralDebug(PERIPHERALS.DEBUG.laserSensor, "Laser sensor") end },
    { name = "Optical sensor", run = function() runPeripheralDebug(PERIPHERALS.DEBUG.opticalSensor, "Optical sensor") end },
    { name = "Gimbal sensor", run = function() runPeripheralDebug(PERIPHERALS.DEBUG.gimbalSensor, "Gimbal sensor") end },
    { name = "Navigation table", run = function() runPeripheralDebug(PERIPHERALS.DEBUG.navigationTable, "Navigation table") end },
    { name = "Stabilizer servo", run = runStabilizerServoDebug },
}

local function showMenu()
    term.clear()
    term.setCursorPos(1, 1)
    print("=== Debug Menu ===")
    print()
    for i, program in ipairs(PROGRAMS) do
        print(string.format("  %d) %s", i, program.name))
    end
    print()
    print("Select a program (1-" .. #PROGRAMS .. "), or Q to quit:")
end

local function selectProgram()
    while true do
        showMenu()
        local input = read()
        if input:lower() == "q" then
            return nil
        end
        local index = tonumber(input)
        if index and PROGRAMS[index] then
            return PROGRAMS[index]
        end
        print("Invalid selection.")
        sleep(1)
    end
end

local program = selectProgram()
if program then
    program.run()
end
