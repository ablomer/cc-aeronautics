-- Standalone debug menu. Run this directly (NOT through main.lua) to pick a
-- debug program from a list and run it. Press Ctrl+T to stop whichever
-- program is running and return to the shell.

require("config")

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
            local ok, value = pcall(sensor[name])
            if ok then
                print(string.format("%-20s %s", name, tostring(value)))
            else
                print(string.format("%-20s <error: %s>", name, tostring(value)))
            end
        end

        sleep(0.2)
    end
end

-- Add new debug programs here as { name = "...", run = function ... end }.
local PROGRAMS = {
    { name = "Laser sensor", run = function() runPeripheralDebug(PERIPHERALS.DEBUG.laserSensor, "Laser sensor") end },
    { name = "Optical sensor", run = function() runPeripheralDebug(PERIPHERALS.DEBUG.opticalSensor, "Optical sensor") end },
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
