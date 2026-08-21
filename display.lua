-- Display layout, top to bottom:
--   1-3   header / fault banner
--   4-11  LNAV  -- everything that drives forward speed and heading: the
--                  propeller-mixer system (velocity, throttle, P1/P2) and
--                  steering (manual wheel or nav autopilot, heading,
--                  bearing, steer output, distance to target). Grouped
--                  together because on this ship both speed and steering
--                  are ultimately commands into the same propeller mixer
--                  (see main.lua's rightMixer/leftMixer), so there isn't a
--                  meaningful "lateral-only" subset separate from speed.
--   12-16 VNAV  -- everything that drives vertical motion: altitude hold,
--                  vertical speed, burner amount, the always-on lift
--                  propeller assist, ground clearance, and landing status.
--   17-18 control-fault detail (blank in normal operation)
--   19    bottom border
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
local COL_LNAV     = colors.cyan
local COL_VNAV     = colors.orange
local COL_ERROR    = colors.red

local BORDER_TOP = "\xc9" .. string.rep("\xcd", W - 2) .. "\xbb"
local BORDER_MID = "\xcc" .. string.rep("\xcd", W - 2) .. "\xb9"
local BORDER_BTM = "\xc8" .. string.rep("\xcd", W - 2) .. "\xbc"
local BORDER_ROW = "\xba" .. string.rep(" ",    W - 2) .. "\xba"

-- Button hit regions (x1, x2, y) — only active in hold mode
local BTN_DEC   = { x1 = 26, x2 = 30, y = 7 }  -- [ - ]
local BTN_INC   = { x1 = 32, x2 = 36, y = 7 }  -- [ + ]
local BTN_ZERO  = { x1 = 38, x2 = 42, y = 7 }  -- [ 0 ]
local BTN_TWO   = { x1 = 44, x2 = 48, y = 7 }  -- [ 2 ]

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

-- Draws a section-divider row with a short label embedded near the left
-- edge (e.g. "LNAV"), instead of a plain blank divider -- this is the
-- visual anchor that separates the two nav domains on the display.
local function drawSectionDivider(term, y, label, labelColor)
    drawBorderLine(term, y, BORDER_MID)
    writeAt(term, 3, y, " " .. label .. " ", labelColor, COL_BG)
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
        writeAt(t, math.floor((W - #title) / 2) + 1, 2, title, COL_ERROR)
    else
        local title = "FLIGHT COMPUTER"
        writeAt(t, math.floor((W - #title) / 2) + 1, 2, title, COL_HEADER)
    end

    -- Row 3: divider
    drawBorderLine(t, 3, BORDER_MID)

    ------------------------------------------------------------------
    -- LNAV: speed + steering (rows 4-11)
    ------------------------------------------------------------------

    -- Row 4: section label
    drawSectionDivider(t, 4, "LNAV", COL_LNAV)

    -- Row 5: velocity mode / steer mode
    drawRow(t, 5)
    writeLabel(t, 3, 5, "VEL MODE")
    if state.holdMode then
        writeAt(t, 14, 5, "[ HOLD ]  ", COL_HOLD)
    else
        writeAt(t, 14, 5, "[ MANUAL ]", COL_MANUAL)
    end
    writeLabel(t, 26, 5, "STEER MODE")
    if state.navSteering then
        writeAt(t, 38, 5, "[ AUTO ]  ", COL_NAV)
    else
        writeAt(t, 38, 5, "[ MANUAL ]", COL_MANUAL)
    end

    -- Row 6: current velocity / throttle
    drawRow(t, 6)
    writeLabel(t, 3, 6, "VELOCITY")
    writeAt(t, 14, 6, string.format("%-10s", string.format("%.2f m/s", state.velocity)))
    writeLabel(t, 26, 6, "THROTTLE")
    writeAt(t, 36, 6, string.format("%-8s", string.format("%d/15", state.throttle)))

    -- Row 7: target velocity with buttons (hold mode only)
    drawRow(t, 7)
    writeLabel(t, 3, 7, "TARGET  ")
    if state.holdMode and state.targetVelocity ~= nil then
        writeAt(t, 14, 7, string.format("%-10s", string.format("%.2f m/s", state.targetVelocity)), COL_HOLD)
        drawButton(t, BTN_DEC.x1,  7, "[ - ]")
        drawButton(t, BTN_INC.x1,  7, "[ + ]")
        drawButton(t, BTN_ZERO.x1, 7, "[ 0 ]")
        drawButton(t, BTN_TWO.x1,  7, "[ 2 ]")
    else
        writeAt(t, 14, 7, string.format("%-10s", "-- manual"), COL_LABEL)
    end

    -- Row 8: propeller power
    drawRow(t, 8)
    writeLabel(t, 3, 8, "PROP P1")
    writeAt(t, 14, 8, string.format("%-10s", string.format("%.1f / 15", state.propeller1Power or 0)))
    writeLabel(t, 26, 8, "PROP P2")
    writeAt(t, 36, 8, string.format("%-8s", string.format("%.1f/15", state.propeller2Power or 0)))

    -- Row 9: steering angle / heading
    drawRow(t, 9)
    writeLabel(t, 3, 9, "STEER   ")
    writeAt(t, 14, 9, string.format("%-10s", string.format("%.1f deg", state.steeringAngle or 0)))
    writeLabel(t, 26, 9, "HEADING")
    if state.navActive then
        writeAt(t, 36, 9, string.format("%-8s", string.format("%.1f", state.navHeading or 0)), COL_NAV)
    else
        writeAt(t, 36, 9, string.format("%-8s", "-"), COL_LABEL)
    end

    -- Row 10: bearing / steer output
    drawRow(t, 10)
    writeLabel(t, 3, 10, "BEARING ")
    if state.navActive then
        writeAt(t, 14, 10, string.format("%-10s", string.format("%.1f deg", state.navBearing or 0)), COL_NAV)
    else
        writeAt(t, 14, 10, string.format("%-10s", "-"), COL_LABEL)
    end
    writeLabel(t, 26, 10, "STEER OUT")
    if state.navActive then
        writeAt(t, 36, 10, string.format("%-8s", string.format("%.2f", state.navOutput or 0)), COL_NAV)
    else
        writeAt(t, 36, 10, string.format("%-8s", "-"), COL_LABEL)
    end

    -- Row 11: distance to nav target
    drawRow(t, 11)
    writeLabel(t, 3, 11, "DISTANCE")
    if state.navActive then
        writeAt(t, 14, 11, string.format("%-10s", string.format("%.1f m", state.navDistance or 0)), COL_NAV)
    else
        writeAt(t, 14, 11, string.format("%-10s", "-"), COL_LABEL)
    end

    ------------------------------------------------------------------
    -- VNAV: altitude + vertical motion (rows 12-16)
    ------------------------------------------------------------------

    -- Row 12: section label
    drawSectionDivider(t, 12, "VNAV", COL_VNAV)

    -- Row 13: current / target altitude
    drawRow(t, 13)
    local altColor = state.altitudeFault and COL_ERROR or COL_VALUE
    writeLabel(t, 3, 13, "ALTITUDE")
    writeAt(t, 14, 13, string.format("%-10s", string.format("%.1f m", state.altitude or 0)), altColor)
    -- TARGET is shown in orange, with a "*" marker, whenever ground
    -- protection has raised the effective target above what the pilot
    -- actually requested -- this is the one place the operator can see that
    -- the autopilot is overriding the lever, not just following it.
    local targetColor = state.groundProtectionActive and colors.orange or COL_HOLD
    local targetLabel = state.groundProtectionActive and "TARGET*" or "TARGET "
    writeLabel(t, 26, 13, targetLabel)
    writeAt(t, 34, 13, string.format("%-10s", string.format("%.1f m", state.targetAltitude or 0)), targetColor)

    -- Row 14: vertical speed / burner amount
    drawRow(t, 14)
    writeLabel(t, 3, 14, "VSPD    ")
    writeAt(t, 14, 14, string.format("%-10s", string.format("%.2f m/s", state.verticalSpeed or 0)))
    writeLabel(t, 26, 14, "BURNER")
    writeAt(t, 34, 14, string.format("%-10s", string.format("%.0f", state.burnerAmount or 0)))

    -- Row 15: lift propeller / ground clearance
    drawRow(t, 15)
    writeLabel(t, 3, 15, "LIFT PROP")
    local liftColor = (state.liftPropellerPower or 0) > 0 and colors.orange or COL_VALUE
    writeAt(t, 14, 15, string.format("%-10s", string.format("%.1f / 15", state.liftPropellerPower or 0)), liftColor)
    writeLabel(t, 26, 15, "GND")
    if state.groundFault then
        writeAt(t, 34, 15, string.format("%-10s", "FAULT"), COL_ERROR)
    else
        local gndColor = (state.groundDistance and state.groundDistance < 15) and colors.orange or COL_VALUE
        writeAt(t, 34, 15, string.format("%-10s", string.format("%.1f m", state.groundDistance or 15)), gndColor)
    end

    -- Row 16: landing status. No button anymore -- landing is driven
    -- entirely by the burner lever (position 0), see landing.lua/main.lua.
    -- This is a read-only status readout, not a control.
    drawRow(t, 16)
    writeLabel(t, 3, 16, "LANDING ")
    if state.landingState == "SLOWING" then
        -- SLOWING's whole purpose is waiting for horizontal speed to die
        -- down (the ship can only coast, not brake -- see landing.lua), so
        -- show that speed here as the reason descent hasn't begun yet.
        writeAt(t, 14, 16, string.format("%-30s", string.format("SLOWING (%.1f m/s)", state.horizontalSpeed or 0)), colors.orange)
    elseif state.landingState == "DESCENDING" then
        writeAt(t, 14, 16, string.format("%-30s", "DESCENDING"), colors.orange)
    elseif state.landingState == "LANDED" then
        writeAt(t, 14, 16, string.format("%-30s", "LANDED"), colors.green)
    else
        writeAt(t, 14, 16, string.format("%-30s", "-"), COL_LABEL)
    end

    ------------------------------------------------------------------
    -- Control-fault detail (rows 17-18), blank in normal operation
    ------------------------------------------------------------------

    drawRow(t, 17)
    drawRow(t, 18)
    if state.controlFault and state.controlError then
        -- Usable width per line: columns 3-49 (47 chars).
        local msg = state.controlError
        local line1 = msg:sub(1, 47)
        local line2 = msg:sub(48, 94)
        writeAt(t, 3, 17, string.format("%-47s", line1), COL_ERROR)
        writeAt(t, 3, 18, string.format("%-47s", line2), COL_ERROR)
    end

    -- Row 19: bottom border
    drawBorderLine(t, 19, BORDER_BTM)
end
