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

local LNAV_MODE = {
    stop = { label = "[ STOP ]   ", color = COL_MANUAL },
    hold = { label = "[ HOLD ]   ", color = COL_HOLD },
    nav  = { label = "[ NAV ]    ", color = COL_NAV },
}

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
    --   lnavMode: "stop"|"hold"|"nav"
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

    -- Row 4: LNAV mode + speed target (mirrors the VNAV header row)
    drawRow(t, 4)
    writeLabel(t, 3, 4, "LNAV")
    local lnav = LNAV_MODE[state.lnavMode] or LNAV_MODE.hold
    writeAt(t, 12, 4, lnav.label, lnav.color)
    writeLabel(t, 26, 4, "TARGET")
    if state.lnavMode == "stop" then
        writeAt(t, 34, 4, string.format("%-10s", "--"), COL_MANUAL)
    else
        writeAt(t, 34, 4, string.format("%-10s", string.format("%.2f m/s", state.targetVelocity or 0)), COL_HOLD)
    end

    -- Row 5: actual velocity + wheel
    drawRow(t, 5)
    writeLabel(t, 3, 5, "VEL     ")
    writeAt(t, 14, 5, string.format("%-10s", string.format("%.2f m/s", state.velocity or 0)))
    writeLabel(t, 26, 5, "WHEEL ")
    writeAt(t, 34, 5, string.format("%-10s", string.format("%.1f deg", state.steeringAngle or 0)))

    -- Row 6: propeller powers
    drawRow(t, 6)
    writeLabel(t, 3, 6, "P1      ")
    writeAt(t, 14, 6, string.format("%-10s", string.format("%.1f", state.propeller1Power or 0)))
    writeLabel(t, 26, 6, "P2    ")
    writeAt(t, 34, 6, string.format("%-10s", string.format("%.1f", state.propeller2Power or 0)))

    -- Row 7: VNAV divider
    drawBorderLine(t, 7, BORDER_MID)

    -- Row 8: VNAV mode + altitude target
    drawRow(t, 8)
    writeLabel(t, 3, 8, "VNAV")
    local vnav = VNAV_MODE[state.vnavMode] or VNAV_MODE.hold
    writeAt(t, 12, 8, vnav.label, vnav.color)
    writeLabel(t, 26, 8, "TARGET")
    if state.landing then
        writeAt(t, 34, 8, string.format("%-10s", "--"), COL_MANUAL)
    else
        writeAt(t, 34, 8, string.format("%-10s", string.format("%.1f m", state.targetAltitude or 0)), COL_HOLD)
    end

    -- Row 9: current altitude / vertical speed
    drawRow(t, 9)
    local altColor = state.altitudeFault and colors.red or COL_VALUE
    writeLabel(t, 3, 9, "ALT     ")
    writeAt(t, 14, 9, string.format("%-10s", string.format("%.1f m", state.altitude or 0)), altColor)
    writeLabel(t, 26, 9, "VSPD  ")
    writeAt(t, 34, 9, string.format("%-10s", string.format("%.2f m/s", state.verticalSpeed or 0)))

    -- Row 10: burner / AGL / desired VS / vertical prop power
    drawRow(t, 10)
    writeLabel(t, 3, 10, "BURN")
    writeAt(t, 8, 10, string.format("%-5s", string.format("%.0f", state.burnerAmount or 0)))
    writeLabel(t, 14, 10, "AGL")
    if state.agl ~= nil then
        writeAt(t, 18, 10, string.format("%-7s", string.format("%.1f m", state.agl)))
    else
        writeAt(t, 18, 10, string.format("%-7s", "--"))
    end
    writeLabel(t, 26, 10, "DVS")
    writeAt(t, 30, 10, string.format("%-7s", string.format("%.2f", state.desiredVS or 0)))
    writeLabel(t, 38, 10, "VP")
    writeAt(t, 41, 10, string.format("%-6s", string.format("%.1f", state.verticalPropPower or 0)))

    -- Rows 11+: nav when a table target exists
    if state.navActive then
        drawBorderLine(t, 11, BORDER_MID)

        drawRow(t, 12)
        writeLabel(t, 3, 12, "HDG     ")
        writeAt(t, 14, 12, string.format("%-10s", string.format("%.1f deg", state.navHeading or 0)), COL_NAV)
        writeLabel(t, 26, 12, "BRG   ")
        writeAt(t, 34, 12, string.format("%-10s", string.format("%.1f deg", state.navBearing or 0)), COL_NAV)

        drawRow(t, 13)
        writeLabel(t, 3, 13, "DIST    ")
        writeAt(t, 14, 13, string.format("%-10s", string.format("%.1f m", state.navDistance or 0)), COL_NAV)
        writeLabel(t, 26, 13, "STEER ")
        writeAt(t, 34, 13, string.format("%-10s", string.format("%.2f", state.navOutput or 0)), COL_NAV)

        drawBorderLine(t, 14, BORDER_BTM)
        for y = 15, 19 do
            drawRow(t, y)
        end
    else
        drawBorderLine(t, 11, BORDER_BTM)
        for y = 12, 19 do
            drawRow(t, y)
        end
    end
end
