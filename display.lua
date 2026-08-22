FlightDisplay = {}

local W, H = 51, 19

local COL_BG       = colors.black
local COL_BORDER   = colors.gray
local COL_HEADER   = colors.white
local COL_LABEL    = colors.lightGray
local COL_VALUE    = colors.white
local COL_MANUAL   = colors.yellow
local COL_HOLD     = colors.green
local COL_NAV      = colors.cyan
local COL_TERRAIN  = colors.orange
local COL_FLARE    = colors.cyan

local VNAV_MODE = {
    hold    = { label = "[ HOLD ]   ", color = COL_HOLD },
    land    = { label = "[ LAND ]   ", color = COL_MANUAL },
    flare   = { label = "[ FLARE ]  ", color = COL_FLARE },
    landed  = { label = "[ LANDED ] ", color = COL_HOLD },
    terrain = { label = "[ TERRAIN ]", color = COL_TERRAIN },
}

local BORDER_TOP = "\xc9" .. string.rep("\xcd", W - 2) .. "\xbb"
local BORDER_MID = "\xcc" .. string.rep("\xcd", W - 2) .. "\xb9"
local BORDER_BTM = "\xc8" .. string.rep("\xcd", W - 2) .. "\xbc"
local BORDER_ROW = "\xba" .. string.rep(" ",    W - 2) .. "\xba"

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

function FlightDisplay:new()
    local t = setmetatable({}, { __index = FlightDisplay })
    t.term = term
    t.term.setBackgroundColor(COL_BG)
    t.term.clear()
    t.term.setCursorBlink(false)
    return t
end

function FlightDisplay:update(state)
    -- state = {
    --   velocity: number (m/s)
    --   throttle: number (0-15 lever notch)
    --   targetVelocity: number (m/s, from the throttle lever)
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
    --   targetAltitude: number (target height, m; unused when landing)
    --   landing: boolean (burner lever detent 0)
    --   vnavMode: "hold"|"land"|"flare"|"landed"|"terrain"
    --   verticalSpeed: number (m/s)
    --   desiredVS: number (commanded vertical speed, m/s)
    --   agl: number|nil (worst-case optical AGL, m; nil when no sensor hasHit)
    --   verticalPropPower: number (last power sent to the vertical prop bank)
    --   burnerAmount: number (last commanded burner amount, 5-500)
    --   altitudeFault: boolean (true when the altitude sensor reading looked invalid)
    -- }

    local t = self.term
    t.setBackgroundColor(COL_BG)

    -- Row 1: top border
    drawBorderLine(t, 1, BORDER_TOP)

    -- Row 2: header
    drawRow(t, 2)
    local title = "FLIGHT COMPUTER"
    writeAt(t, math.floor((W - #title) / 2) + 1, 2, title, COL_HEADER)

    -- Row 3: divider
    drawBorderLine(t, 3, BORDER_MID)

    -- Row 4: blank
    drawRow(t, 4)

    -- Row 5: throttle mode / steering mode
    drawRow(t, 5)
    writeLabel(t, 3, 5, "VELOCITY")
    writeAt(t, 12, 5, "[ HOLD ]  ", COL_HOLD)
    writeLabel(t, 24, 5, "STEER")
    if state.navSteering then
        writeAt(t, 30, 5, "[ AUTO ]  ", COL_NAV)
    else
        writeAt(t, 30, 5, "[ MANUAL ]", COL_MANUAL)
    end

    -- Row 6: blank
    drawRow(t, 6)

    -- Row 7: velocity
    drawRow(t, 7)
    writeLabel(t, 3, 7, "VELOCITY")
    writeAt(t, 14, 7, string.format("%-10s", string.format("%.2f m/s", state.velocity)))

    -- Row 8: target velocity from the throttle lever
    drawRow(t, 8)
    writeLabel(t, 3, 8, "TARGET  ")
    writeAt(t, 14, 8, string.format("%-10s", string.format("%.2f m/s", state.targetVelocity or 0)), COL_HOLD)

    -- Row 9: throttle
    drawRow(t, 9)
    writeLabel(t, 3, 9, "THROTTLE")
    writeAt(t, 14, 9, string.format("%-8s", string.format("%d / 15", state.throttle)))
    writeLabel(t, 22, 9, "P1")
    writeAt(t, 27, 9, string.format("%-8s", string.format("%.1f", state.propeller1Power or 0)))
    writeLabel(t, 34, 9, "P2")
    writeAt(t, 39, 9, string.format("%-8s", string.format("%.1f", state.propeller2Power or 0)))

    -- Row 10: steering angle (debug)
    drawRow(t, 10)
    writeLabel(t, 3, 10, "STEERING")
    writeAt(t, 14, 10, string.format("%-10s", string.format("%.1f deg", state.steeringAngle or 0)))

    -- Row 11: altitude section divider
    drawBorderLine(t, 11, BORDER_MID)

    -- Row 12: VNAV mode chip (same style as VELOCITY / STEER on row 5) + target
    drawRow(t, 12)
    writeLabel(t, 3, 12, "VNAV")
    local mode = VNAV_MODE[state.vnavMode] or VNAV_MODE.hold
    writeAt(t, 12, 12, mode.label, mode.color)
    writeLabel(t, 26, 12, "TARGET")
    if state.landing then
        writeAt(t, 34, 12, string.format("%-10s", "--"), COL_MANUAL)
    else
        writeAt(t, 34, 12, string.format("%-10s", string.format("%.1f m", state.targetAltitude or 0)), COL_HOLD)
    end

    -- Row 13: current altitude / vertical speed
    drawRow(t, 13)
    local altColor = state.altitudeFault and colors.red or COL_VALUE
    writeLabel(t, 3, 13, "ALTITUDE")
    writeAt(t, 14, 13, string.format("%-10s", string.format("%.1f m", state.altitude or 0)), altColor)
    writeLabel(t, 26, 13, "VSPD  ")
    writeAt(t, 34, 13, string.format("%-10s", string.format("%.2f m/s", state.verticalSpeed or 0)))

    -- Row 14: burner / AGL / desired VS / vertical prop power
    drawRow(t, 14)
    writeLabel(t, 3, 14, "BURN")
    writeAt(t, 8, 14, string.format("%-5s", string.format("%.0f", state.burnerAmount or 0)))
    writeLabel(t, 14, 14, "AGL")
    if state.agl ~= nil then
        writeAt(t, 18, 14, string.format("%-7s", string.format("%.1f m", state.agl)))
    else
        writeAt(t, 18, 14, string.format("%-7s", "--"))
    end
    writeLabel(t, 26, 14, "DVS")
    writeAt(t, 30, 14, string.format("%-7s", string.format("%.2f", state.desiredVS or 0)))
    writeLabel(t, 38, 14, "VP")
    writeAt(t, 41, 14, string.format("%-6s", string.format("%.1f", state.verticalPropPower or 0)))

    -- Rows 15+: nav divider and status (when nav active)
    if state.navActive then
        drawBorderLine(t, 15, BORDER_MID)

        -- Row 16: nav heading
        drawRow(t, 16)
        writeLabel(t, 3, 16, "HEADING ")
        writeAt(t, 14, 16, string.format("%-10s", string.format("%.1f deg", state.navHeading or 0)), COL_NAV)

        -- Row 17: nav bearing
        drawRow(t, 17)
        writeLabel(t, 3, 17, "BEARING ")
        writeAt(t, 14, 17, string.format("%-10s", string.format("%.1f deg", state.navBearing or 0)), COL_NAV)

        -- Row 18: steer output and distance (packed to keep the 19-row frame)
        drawRow(t, 18)
        writeLabel(t, 3, 18, "STEER OUT")
        writeAt(t, 14, 18, string.format("%-8s", string.format("%.2f", state.navOutput or 0)), COL_NAV)
        writeLabel(t, 23, 18, "DIST")
        writeAt(t, 28, 18, string.format("%-10s", string.format("%.1f m", state.navDistance or 0)), COL_NAV)

        -- Row 19: bottom border
        drawBorderLine(t, 19, BORDER_BTM)
    else
        -- Row 15: bottom border
        drawBorderLine(t, 15, BORDER_BTM)

        -- Clear any leftover nav rows from previous state
        for y = 16, 19 do
            drawRow(t, y)
        end
    end
end
