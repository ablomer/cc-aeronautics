FlightDisplay = {}

local W, H = 51, 19

local COL_BG       = colors.black
local COL_BORDER   = colors.gray
local COL_HEADER   = colors.white
local COL_LABEL    = colors.lightGray
local COL_VALUE    = colors.white
local COL_MANUAL   = colors.yellow
local COL_HOLD     = colors.green
local COL_BTN_BG   = colors.gray
local COL_BTN_TEXT = colors.white
local COL_NAV      = colors.cyan

local BORDER_TOP = "\xc9" .. string.rep("\xcd", W - 2) .. "\xbb"
local BORDER_MID = "\xcc" .. string.rep("\xcd", W - 2) .. "\xb9"
local BORDER_BTM = "\xc8" .. string.rep("\xcd", W - 2) .. "\xbc"
local BORDER_ROW = "\xba" .. string.rep(" ",    W - 2) .. "\xba"

-- Button hit regions (x1, x2, y) — only active in hold mode
local BTN_DEC   = { x1 = 26, x2 = 30, y = 8 }  -- [ - ]
local BTN_INC   = { x1 = 32, x2 = 36, y = 8 }  -- [ + ]
local BTN_ZERO  = { x1 = 38, x2 = 42, y = 8 }  -- [ 0 ]
local BTN_TWO   = { x1 = 44, x2 = 48, y = 8 }  -- [ 2 ]

local function drawBorderLine(term, y, line)
    term.setCursorPos(1, y)
    term.setTextColor(COL_BORDER)
    term.setBackgroundColor(COL_BG)
    term.write(line)
end

local function drawRow(term, y)
    drawBorderLine(term, y, BORDER_ROW)
end

local function writeAt(term, x, y, text, textColor, bgColor)
    term.setCursorPos(x, y)
    term.setTextColor(textColor or COL_VALUE)
    term.setBackgroundColor(bgColor or COL_BG)
    term.write(text)
end

local function writeLabel(term, x, y, label)
    writeAt(term, x, y, label, COL_LABEL)
end

local function drawButton(term, x, y, label)
    writeAt(term, x, y, label, COL_BTN_TEXT, COL_BTN_BG)
    -- restore background after button
    term.setBackgroundColor(COL_BG)
end

function FlightDisplay:new()
    local t = setmetatable({}, { __index = FlightDisplay })
    t.term = term
    t.term.setBackgroundColor(COL_BG)
    t.term.clear()
    t.term.setCursorBlink(false)
    return t
end

-- Returns the button action for a click, or nil.
function FlightDisplay:hitTest(x, y)
    if x >= BTN_DEC.x1  and x <= BTN_DEC.x2  and y == BTN_DEC.y  then return "dec"  end
    if x >= BTN_INC.x1  and x <= BTN_INC.x2  and y == BTN_INC.y  then return "inc"  end
    if x >= BTN_ZERO.x1 and x <= BTN_ZERO.x2 and y == BTN_ZERO.y then return "zero" end
    if x >= BTN_TWO.x1  and x <= BTN_TWO.x2  and y == BTN_TWO.y  then return "two"  end
    return nil
end

function FlightDisplay:update(state)
    -- state = {
    --   velocity: number (m/s)
    --   throttle: number (0-15)
    --   holdMode: boolean
    --   targetVelocity: number|nil (only relevant in hold mode)
    --   steeringAngle: number (-180 to 180, degrees)
    --   navActive: boolean
    --   navBearing: number|nil (degrees, only when navActive)
    --   navHeading: number|nil (degrees, only when navActive)
    --   navOutput: number|nil (steering output in [-1,1], only when navActive)
    --   navDistance: number|nil (metres, only when navActive)
    --   navSteering: boolean (true when autopilot is controlling steering)
    --   propeller1Power: number (last power sent to propeller 1)
    --   propeller2Power: number (last power sent to propeller 2)
    --   altitude: number (current height, m)
    --   targetAltitude: number (target height, m)
    --   verticalSpeed: number (m/s)
    --   burnerAmount: number (last commanded burner amount, 5-500)
    --   altitudeFault: boolean (true when the altitude sensor reading looked invalid)
    --   groundDistance: number|nil (metres, 0-15, minimum reading across optical sensors)
    --   groundSensor: string|nil (name of the sensor reporting groundDistance)
    --   groundFault: boolean (true when no optical sensor is giving a healthy reading)
    --   groundProtectionActive: boolean (true when ground protection is overriding the pilot's requested altitude target this tick)
    --   liftPropellerPower: number (last power sent to the bottom lift-propeller bank, 0-15)
    --   landingState: string ("IDLE" | "SLOWING" | "DESCENDING" | "LANDED") -- driven by the burner lever (position 0), see landing.lua
    --   horizontalSpeed: number|nil (m/s, forward velocity -- used to show why landing is still in SLOWING)
    --   controlFault: boolean|nil (true when the control loop itself errored this tick)
    --   controlError: string|nil (error message when controlFault is true)
    -- }

    local t = self.term
    t.setBackgroundColor(COL_BG)

    -- Row 1: top border
    drawBorderLine(t, 1, BORDER_TOP)

    -- Row 2: header. Overridden to a visible alarm when the control loop
    -- itself faulted (see main.lua's safeControlUpdate) -- this is the one
    -- state where the operator needs to know the autopilot stopped running.
    drawRow(t, 2)
    if state.controlFault then
        local title = "CONTROL FAULT - BURNER MIN / LIFT MAX"
        writeAt(t, math.floor((W - #title) / 2) + 1, 2, title, colors.red)
    else
        local title = "FLIGHT COMPUTER"
        writeAt(t, math.floor((W - #title) / 2) + 1, 2, title, COL_HEADER)
    end

    -- Row 3: divider
    drawBorderLine(t, 3, BORDER_MID)

    -- Row 4: blank
    drawRow(t, 4)

    -- Row 5: throttle mode / steering mode
    drawRow(t, 5)
    writeLabel(t, 3, 5, "VELOCITY")
    if state.holdMode then
        writeAt(t, 12, 5, "[ HOLD ]  ", COL_HOLD)
    else
        writeAt(t, 12, 5, "[ MANUAL ]", COL_MANUAL)
    end
    writeLabel(t, 24, 5, "STEER")
    if state.navSteering then
        writeAt(t, 30, 5, "[ AUTO ]  ", COL_NAV)
    else
        writeAt(t, 30, 5, "[ MANUAL ]", COL_MANUAL)
    end

    -- Row 6: lift propeller (always-on ground-protection assist) / landing control
    drawRow(t, 6)
    writeLabel(t, 3, 6, "LIFT PROP")
    local liftColor = (state.liftPropellerPower or 0) > 0 and colors.orange or COL_VALUE
    writeAt(t, 14, 6, string.format("%-10s", string.format("%.1f / 15", state.liftPropellerPower or 0)), liftColor)

    -- Landing status: no button anymore -- landing is driven entirely by
    -- the burner lever (position 0), see landing.lua/main.lua. This is a
    -- read-only status readout, not a control.
    writeLabel(t, 26, 6, "LAND")
    if state.landingState == "SLOWING" then
        -- SLOWING's whole purpose is waiting for horizontal speed to
        -- die down (the ship can only coast, not brake -- see landing.lua),
        -- so show that speed here as the reason descent hasn't begun yet.
        writeAt(t, 31, 6, string.format("%-14s", string.format("SLOWING %.1fm/s", state.horizontalSpeed or 0)), colors.orange)
    elseif state.landingState == "DESCENDING" then
        writeAt(t, 31, 6, string.format("%-14s", "DESCENDING"), colors.orange)
    elseif state.landingState == "LANDED" then
        writeAt(t, 31, 6, string.format("%-14s", "LANDED"), colors.green)
    else
        writeAt(t, 31, 6, string.format("%-14s", "-"), COL_LABEL)
    end

    -- Row 7: velocity
    drawRow(t, 7)
    writeLabel(t, 3, 7, "VELOCITY")
    writeAt(t, 14, 7, string.format("%-10s", string.format("%.2f m/s", state.velocity)))

    -- Row 8: target velocity with buttons (hold mode only) or blank
    drawRow(t, 8)
    if state.holdMode and state.targetVelocity ~= nil then
        writeLabel(t, 3, 8, "TARGET  ")
        writeAt(t, 14, 8, string.format("%-10s", string.format("%.2f m/s", state.targetVelocity)), COL_HOLD)
        drawButton(t, BTN_DEC.x1,  8, "[ - ]")
        drawButton(t, BTN_INC.x1,  8, "[ + ]")
        drawButton(t, BTN_ZERO.x1, 8, "[ 0 ]")
        drawButton(t, BTN_TWO.x1,  8, "[ 2 ]")
    end

    -- Row 9: throttle
    drawRow(t, 9)
    writeLabel(t, 3, 9, "THROTTLE")
    writeAt(t, 14, 9, string.format("%-8s", string.format("%d / 15", state.throttle)))
    writeLabel(t, 22, 9, "P1")
    writeAt(t, 27, 9, string.format("%-8s", string.format("%.1f", state.propeller1Power or 0)))
    writeLabel(t, 34, 9, "P2")
    writeAt(t, 39, 9, string.format("%-8s", string.format("%.1f", state.propeller2Power or 0)))

    -- Row 10: steering angle (debug) / ground clearance (optical sensor bank)
    drawRow(t, 10)
    writeLabel(t, 3, 10, "STEERING")
    writeAt(t, 14, 10, string.format("%-10s", string.format("%.1f deg", state.steeringAngle or 0)))
    writeLabel(t, 26, 10, "GND")
    if state.groundFault then
        writeAt(t, 34, 10, string.format("%-10s", "FAULT"), colors.red)
    else
        local gndColor = (state.groundDistance and state.groundDistance < 15) and colors.orange or COL_VALUE
        writeAt(t, 34, 10, string.format("%-10s", string.format("%.1f m", state.groundDistance or 15)), gndColor)
    end

    -- Row 11: altitude section divider
    drawBorderLine(t, 11, BORDER_MID)

    -- Row 12: current / target altitude
    drawRow(t, 12)
    local altColor = state.altitudeFault and colors.red or COL_VALUE
    writeLabel(t, 3, 12, "ALTITUDE")
    writeAt(t, 14, 12, string.format("%-10s", string.format("%.1f m", state.altitude or 0)), altColor)
    -- TARGET is shown in orange, with a "*" marker, whenever ground
    -- protection has raised the effective target above what the pilot
    -- actually requested -- this is the one place the operator can see that
    -- the autopilot is overriding the lever, not just following it.
    local targetColor = state.groundProtectionActive and colors.orange or COL_HOLD
    local targetLabel = state.groundProtectionActive and "TARGET*" or "TARGET "
    writeLabel(t, 26, 12, targetLabel)
    writeAt(t, 34, 12, string.format("%-10s", string.format("%.1f m", state.targetAltitude or 0)), targetColor)

    -- Row 13: burner amount / vertical speed
    drawRow(t, 13)
    writeLabel(t, 3, 13, "BURNER  ")
    writeAt(t, 14, 13, string.format("%-10s", string.format("%.0f", state.burnerAmount or 0)))
    writeLabel(t, 26, 13, "VSPD  ")
    writeAt(t, 34, 13, string.format("%-10s", string.format("%.2f m/s", state.verticalSpeed or 0)))

    -- Rows 14+: nav divider and status (when nav active)
    if state.navActive then
        drawBorderLine(t, 14, BORDER_MID)

        -- Row 15: nav heading
        drawRow(t, 15)
        writeLabel(t, 3, 15, "HEADING ")
        writeAt(t, 14, 15, string.format("%-10s", string.format("%.1f deg", state.navHeading or 0)), COL_NAV)

        -- Row 16: nav bearing
        drawRow(t, 16)
        writeLabel(t, 3, 16, "BEARING ")
        writeAt(t, 14, 16, string.format("%-10s", string.format("%.1f deg", state.navBearing or 0)), COL_NAV)

        -- Row 17: nav autopilot steering output
        drawRow(t, 17)
        writeLabel(t, 3, 17, "STEER OUT")
        writeAt(t, 14, 17, string.format("%-10s", string.format("%.2f", state.navOutput or 0)), COL_NAV)

        -- Row 18: distance to target
        drawRow(t, 18)
        writeLabel(t, 3, 18, "DISTANCE")
        writeAt(t, 14, 18, string.format("%-10s", string.format("%.1f m", state.navDistance or 0)), COL_NAV)

        -- Row 19: bottom border
        drawBorderLine(t, 19, BORDER_BTM)
    else
        -- Row 14: bottom border
        drawBorderLine(t, 14, BORDER_BTM)

        -- Clear any leftover nav rows from previous state
        for y = 15, 19 do
            drawRow(t, y)
        end
    end
end
