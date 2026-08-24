FlightDisplay = {}

local W, H = 51, 19

local COL_BG       = colors.black
local COL_BORDER   = colors.gray
local COL_HEADER   = colors.white
local COL_LABEL    = colors.lightGray
local COL_VALUE    = colors.white
local COL_MANUAL   = colors.yellow
local COL_HOLD     = colors.green
local COL_TERRAIN  = colors.orange
local COL_FLARE    = colors.cyan

local LNAV_MODE = {
    stop = { label = "[ STOP ]   ", color = COL_MANUAL },
    hold = { label = "[ HOLD ]   ", color = COL_HOLD },
}

local VNAV_MODE = {
    hold    = { label = "[ HOLD ]   ", color = COL_HOLD },
    land    = { label = "[ LAND ]   ", color = COL_MANUAL },
    flare   = { label = "[ FLARE ]  ", color = COL_FLARE },
    landed  = { label = "[ LANDED ] ", color = COL_HOLD },
    terrain = { label = "[ TERRAIN ]", color = COL_TERRAIN },
}

local ATT_MODE = {
    level = { label = "[ LEVEL ]  ", color = COL_HOLD },
    off   = { label = "[ OFF ]    ", color = COL_MANUAL },
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
    --   lnavMode: "stop"|"hold"
    --   heading: number|nil (0-360, 0 = north)
    --   steerSource: "NAV"|"WHEEL"
    --   relativeBearing: number (deg, selected heading command)
    --   requestedSpeed: number (normalized speed request 0-1)
    --   requestedSteer: number (normalized steering request -1-1)
    --   requestedCommonRpm: number (forward RPM before mixer reduction)
    --   appliedCommonRpm: number (forward RPM after mixer reduction)
    --   speedReduced: boolean (mixer shed forward thrust for steering)
    --   propeller1Rpm: number (applied left RPM)
    --   propeller2Rpm: number (applied right RPM)
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
    --   pitch: number|nil (gimbal pitch, deg)
    --   pitchRate: number|nil (gimbal pitch rate, deg/s)
    --   stabAngle: number|nil (stabilizer bearing angle, deg)
    --   stabTarget: number (commanded stabilizer angle, deg)
    --   stabRpm: number (last RSC speed)
    --   attMode: "level"|"off"
    --   attFault: boolean
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

    -- Row 4: LNAV mode + heading + steering source
    drawRow(t, 4)
    writeLabel(t, 3, 4, "LNAV")
    local lnav = LNAV_MODE[state.lnavMode] or LNAV_MODE.hold
    writeAt(t, 12, 4, lnav.label, lnav.color)
    writeLabel(t, 24, 4, "HDG")
    if state.heading ~= nil then
        writeAt(t, 28, 4, string.format("%3.0f", state.heading))
    else
        writeAt(t, 28, 4, " --")
    end
    writeLabel(t, 34, 4, "SRC")
    local src = state.steerSource or "WHEEL"
    local srcColor = src == "NAV" and COL_HOLD or COL_MANUAL
    writeAt(t, 38, 4, string.format("%-5s", src), srcColor)

    -- Row 5: actual velocity + speed target + throttle detent
    drawRow(t, 5)
    writeLabel(t, 3, 5, "VEL")
    writeAt(t, 7, 5, string.format("%-8s", string.format("%.2f", state.velocity or 0)))
    writeLabel(t, 16, 5, "TGT")
    if state.lnavMode == "stop" then
        writeAt(t, 20, 5, string.format("%-8s", "--"), COL_MANUAL)
    else
        writeAt(t, 20, 5, string.format("%-8s", string.format("%.2f", state.targetVelocity or 0)), COL_HOLD)
    end
    writeLabel(t, 29, 5, "THR")
    writeAt(t, 33, 5, string.format("%d", state.throttle or 0))

    -- Row 6: requested speed / steering / common-mode RPM
    drawRow(t, 6)
    writeLabel(t, 3, 6, "REQ")
    writeLabel(t, 7, 6, "SPD")
    writeAt(t, 11, 6, string.format("%4.2f", state.requestedSpeed or 0))
    writeLabel(t, 17, 6, "STR")
    writeAt(t, 21, 6, string.format("%+5.2f", state.requestedSteer or 0))
    writeLabel(t, 28, 6, "COM")
    writeAt(t, 32, 6, string.format("%d", math.floor((state.requestedCommonRpm or 0) + 0.5)))

    -- Row 7: applied left/right RPM and common-mode after mixer
    drawRow(t, 7)
    writeLabel(t, 3, 7, "APP")
    writeLabel(t, 7, 7, "L")
    writeAt(t, 9, 7, string.format("%4d", state.propeller1Rpm or 0))
    writeLabel(t, 15, 7, "R")
    writeAt(t, 17, 7, string.format("%4d", state.propeller2Rpm or 0))
    writeLabel(t, 23, 7, "COM")
    local comColor = state.speedReduced and COL_MANUAL or COL_VALUE
    writeAt(t, 27, 7, string.format("%d", math.floor((state.appliedCommonRpm or 0) + 0.5)), comColor)
    if state.speedReduced then
        writeAt(t, 32, 7, "CUT", COL_MANUAL)
    end

    -- Row 8: VNAV divider
    drawBorderLine(t, 8, BORDER_MID)

    -- Row 9: VNAV mode + altitude target
    drawRow(t, 9)
    writeLabel(t, 3, 9, "VNAV")
    local vnav = VNAV_MODE[state.vnavMode] or VNAV_MODE.hold
    writeAt(t, 12, 9, vnav.label, vnav.color)
    writeLabel(t, 26, 9, "TARGET")
    if state.landing then
        writeAt(t, 34, 9, string.format("%-10s", "--"), COL_MANUAL)
    else
        writeAt(t, 34, 9, string.format("%-10s", string.format("%.1f m", state.targetAltitude or 0)), COL_HOLD)
    end

    -- Row 10: current altitude / vertical speed
    drawRow(t, 10)
    local altColor = state.altitudeFault and colors.red or COL_VALUE
    writeLabel(t, 3, 10, "ALT     ")
    writeAt(t, 14, 10, string.format("%-10s", string.format("%.1f m", state.altitude or 0)), altColor)
    writeLabel(t, 26, 10, "VSPD  ")
    writeAt(t, 34, 10, string.format("%-10s", string.format("%.2f m/s", state.verticalSpeed or 0)))

    -- Row 11: burner / AGL / desired VS / vertical prop power
    drawRow(t, 11)
    writeLabel(t, 3, 11, "BURN")
    writeAt(t, 8, 11, string.format("%-5s", string.format("%.0f", state.burnerAmount or 0)))
    writeLabel(t, 14, 11, "AGL")
    if state.agl ~= nil then
        writeAt(t, 18, 11, string.format("%-7s", string.format("%.1f m", state.agl)))
    else
        writeAt(t, 18, 11, string.format("%-7s", "--"))
    end
    writeLabel(t, 26, 11, "DVS")
    writeAt(t, 30, 11, string.format("%-7s", string.format("%.2f", state.desiredVS or 0)))
    writeLabel(t, 38, 11, "VP")
    writeAt(t, 41, 11, string.format("%-6s", string.format("%.1f", state.verticalPropPower or 0)))

    -- Row 12: ATT divider
    drawBorderLine(t, 12, BORDER_MID)

    -- Row 13: ATT mode + pitch
    drawRow(t, 13)
    writeLabel(t, 3, 13, "ATT")
    local att = ATT_MODE[state.attMode] or ATT_MODE.off
    writeAt(t, 12, 13, att.label, att.color)
    writeLabel(t, 26, 13, "PITCH")
    local pitchColor = state.attFault and colors.red or COL_VALUE
    if state.pitch ~= nil then
        writeAt(t, 34, 13, string.format("%-10s", string.format("%.1f deg", state.pitch)), pitchColor)
    else
        writeAt(t, 34, 13, string.format("%-10s", "--"), pitchColor)
    end

    -- Row 14: stabilizer angle / command / RPM
    drawRow(t, 14)
    writeLabel(t, 3, 14, "STAB")
    if state.stabAngle ~= nil then
        writeAt(t, 8, 14, string.format("%-7s", string.format("%.1f", state.stabAngle)))
    else
        writeAt(t, 8, 14, string.format("%-7s", "--"))
    end
    writeLabel(t, 16, 14, "CMD")
    writeAt(t, 20, 14, string.format("%-7s", string.format("%.1f", state.stabTarget or 0)))
    writeLabel(t, 28, 14, "RPM")
    writeAt(t, 32, 14, string.format("%-6s", string.format("%d", state.stabRpm or 0)))

    drawBorderLine(t, 15, BORDER_BTM)
    for y = 16, H do
        drawRow(t, y)
    end
end
