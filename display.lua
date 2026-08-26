FlightDisplay = {}

local W, H = 51, 19

local COL_BG       = colors.black
local COL_BORDER   = colors.gray
local COL_LABEL    = colors.lightGray
local COL_VALUE    = colors.white
local COL_SELECTED = colors.orange
local COL_MANUAL   = colors.yellow
local COL_HOLD     = colors.green
local COL_TERRAIN  = colors.orange
local COL_FLARE    = colors.cyan

local LNAV_MODE = {
    stop = { label = "[STOP]", color = COL_MANUAL },
    hold = { label = "[HOLD]", color = COL_HOLD },
}

local VNAV_MODE = {
    hold    = { label = "[HOLD]",    color = COL_HOLD },
    land    = { label = "[LAND]",    color = COL_MANUAL },
    flare   = { label = "[FLARE]",   color = COL_FLARE },
    landed  = { label = "[LANDED]",  color = COL_HOLD },
    terrain = { label = "[TERRAIN]", color = COL_TERRAIN },
}

local ATT_MODE = {
    level = { label = "[LEVEL]", color = COL_HOLD },
    off   = { label = "[OFF]",   color = COL_MANUAL },
    none  = { label = "[NONE]",  color = COL_LABEL },
}

-- Four FCU windows. Splits are the inner verticals; each WIN is the
-- content range between them (or the outer border). The mode section
-- under the windows is two panes: LNAV under SPD+BRG, VNAV under ALT+HEAT.
local SPLITS = {13, 25, 38}
local MODE_SPLIT = 25
local WIN = {
    {x = 2,  w = 11}, -- SPD
    {x = 14, w = 11}, -- BRG
    {x = 26, w = 12}, -- ALT
    {x = 39, w = 12}, -- HEAT
}
local LNAV_WIN = {x = 2,  w = 23} -- under SPD + BRG
local VNAV_WIN = {x = 26, w = 25} -- under ALT + HEAT

local function makeLine(left, fill, right, junctions)
    local chars = {}
    for i = 1, W do
        chars[i] = fill
    end
    chars[1] = left
    chars[W] = right
    if junctions ~= nil then
        for x, ch in pairs(junctions) do
            chars[x] = ch
        end
    end
    return table.concat(chars)
end

local BORDER_TOP = makeLine("\xc9", "\xcd", "\xbb", {
    [13] = "\xcb", [25] = "\xcb", [38] = "\xcb",
})
-- 4-column FCU closes; the BRG|ALT split continues as LNAV|VNAV.
local BORDER_FCU_MODES = makeLine("\xcc", "\xcd", "\xb9", {
    [13] = "\xca", [25] = "\xce", [38] = "\xca",
})
local BORDER_MODES_DETAIL = makeLine("\xcc", "\xcd", "\xb9", {
    [25] = "\xce",
})
local BORDER_DETAIL_ATT = makeLine("\xcc", "\xcd", "\xb9", {
    [25] = "\xca",
})
local BORDER_BTM = makeLine("\xc8", "\xcd", "\xbc")
local BORDER_ROW = makeLine("\xba", " ",    "\xba")

local function drawBorderLine(term, y, line)
    term.setCursorPos(1, y)
    term.setTextColor(COL_BORDER)
    term.setBackgroundColor(COL_BG)
    term.write(line)
end

local function drawRow(term, y)
    drawBorderLine(term, y, BORDER_ROW)
end

local function drawSplitRow(term, y, splits)
    drawRow(term, y)
    term.setTextColor(COL_BORDER)
    term.setBackgroundColor(COL_BG)
    for i = 1, #splits do
        term.setCursorPos(splits[i], y)
        term.write("\xba")
    end
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

-- Left- or right-aligned text clipped to a window.
local function writeWin(term, win, y, text, color, align)
    text = text or ""
    if #text > win.w then
        text = string.sub(text, 1, win.w)
    end
    local x = win.x
    if align == "right" then
        x = win.x + win.w - #text
    end
    writeAt(term, x, y, text, color)
end

local PAIR_WIDTH = 5

local function fmtPairNum(fmt, value)
    if value == nil then
        return string.format("%-" .. PAIR_WIDTH .. "s", "--")
    end
    return string.format(fmt, value)
end

local function writePair(term, x, y, currentStr, targetStr, currentColor, targetColor)
    writeAt(term, x, y, currentStr, currentColor or COL_VALUE)
    writeAt(term, x + #currentStr, y, " / ", COL_LABEL)
    writeAt(term, x + #currentStr + 3, y, targetStr, targetColor or COL_HOLD)
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
    --   yawRate: number|nil (gimbal wy, deg/s; nil if the reading is unusable)
    --   speedReduced: boolean (mixer shed forward thrust for steering)
    --   propeller1Rpm: number (applied left RPM)
    --   propeller2Rpm: number (applied right RPM)
    --   altitude: number (current height, m)
    --   targetAltitude: number|nil (target height, m; nil when landing)
    --   landing: boolean (burner lever detent 0)
    --   vnavMode: "hold"|"land"|"flare"|"landed"|"terrain"
    --   verticalSpeed: number (m/s, display only)
    --   targetVolume: number|nil (formula / landing heat target, m³)
    --   currentVolume: number (commanded heated volume, m³)
    --   agl: number|nil (worst-case optical AGL, m; nil when no sensor hasHit)
    --   verticalPropRpm: number (last RPM sent to the vertical prop RSC)
    --   balloonCapacity: number (balloon envelope capacity, m³)
    --   volumeSlewRate: number|nil (current heat-command slew limit, m³/s)
    --   altitudeFault: boolean
    --   pitch: number|nil (gimbal pitch, deg)
    --   stabAngle: number|nil (stabilizer bearing angle, deg)
    --   stabTarget: number (commanded stabilizer angle, deg)
    --   attMode: "level"|"off"|"none"
    --   attFault: boolean
    -- }

    local t = self.term
    t.setBackgroundColor(COL_BG)

    local spdWin, brgWin, altWin, heatWin = WIN[1], WIN[2], WIN[3], WIN[4]
    local src = state.steerSource or "WHEEL"

    -- Row 1: FCU top
    drawBorderLine(t, 1, BORDER_TOP)

    -- Row 2: window labels
    drawSplitRow(t, 2, SPLITS)
    writeWin(t, spdWin, 2, " SPD", COL_LABEL, "left")
    writeWin(t, brgWin, 2, " BRG", COL_LABEL, "left")
    writeWin(t, altWin, 2, " ALT", COL_LABEL, "left")
    writeWin(t, heatWin, 2, " HEAT", COL_LABEL, "left")

    -- Row 3: selected / target
    drawSplitRow(t, 3, SPLITS)
    if state.lnavMode == "stop" then
        writeWin(t, spdWin, 3, "--.--", COL_SELECTED, "right")
    else
        writeWin(t, spdWin, 3, string.format("%5.2f", state.targetVelocity or 0), COL_SELECTED, "right")
    end
    if src == "NAV" then
        writeWin(t, brgWin, 3, string.format("%+5.0f", 0), COL_SELECTED, "right")
    else
        writeWin(t, brgWin, 3, "  ---", COL_SELECTED, "right")
    end
    if state.targetAltitude == nil then
        writeWin(t, altWin, 3, "---.-", COL_SELECTED, "right")
    else
        writeWin(t, altWin, 3, string.format("%5.1f", state.targetAltitude), COL_SELECTED, "right")
    end
    if state.targetVolume == nil then
        writeWin(t, heatWin, 3, "  ---", COL_SELECTED, "right")
    else
        writeWin(t, heatWin, 3, string.format("%5.0f", state.targetVolume), COL_SELECTED, "right")
    end

    -- Row 4: actual
    drawSplitRow(t, 4, SPLITS)
    writeWin(t, spdWin, 4, string.format("%5.2f", state.velocity or 0), COL_VALUE, "right")
    writeWin(t, brgWin, 4, string.format("%+5.0f", state.relativeBearing or 0), COL_VALUE, "right")
    local altColor = state.altitudeFault and colors.red or COL_VALUE
    writeWin(t, altWin, 4, string.format("%5.1f", state.altitude or 0), altColor, "right")
    writeWin(t, heatWin, 4, string.format("%5.0f", state.currentVolume or 0), COL_VALUE, "right")

    -- Row 5: close the four windows; LNAV|VNAV split continues
    drawBorderLine(t, 5, BORDER_FCU_MODES)

    -- Row 6: modes — LNAV under SPD+BRG, VNAV under ALT+HEAT
    drawSplitRow(t, 6, {MODE_SPLIT})
    local lnav = LNAV_MODE[state.lnavMode] or LNAV_MODE.hold
    writeLabel(t, LNAV_WIN.x + 1, 6, "LNAV")
    writeAt(t, LNAV_WIN.x + 6, 6, lnav.label, lnav.color)
    local srcColor = src == "NAV" and COL_HOLD or COL_MANUAL
    writeWin(t, LNAV_WIN, 6, src, srcColor, "right")
    local vnav = VNAV_MODE[state.vnavMode] or VNAV_MODE.hold
    writeLabel(t, VNAV_WIN.x + 1, 6, "VNAV")
    writeAt(t, VNAV_WIN.x + 6, 6, vnav.label, vnav.color)
    writeLabel(t, VNAV_WIN.x + 16, 6, "LIM")
    if state.volumeSlewRate == nil then
        writeAt(t, VNAV_WIN.x + 20, 6, "  --")
    else
        writeAt(t, VNAV_WIN.x + 20, 6, string.format("%4.1f", state.volumeSlewRate))
    end

    -- Row 7: modes to LNAV/VNAV detail panes
    drawBorderLine(t, 7, BORDER_MODES_DETAIL)

    -- Row 8: HDG/YAW under LNAV, AGL/capacity under VNAV
    drawSplitRow(t, 8, {MODE_SPLIT})
    local lnavLblL, lnavValL = LNAV_WIN.x + 1, LNAV_WIN.x + 5
    local lnavLblR, lnavValR = LNAV_WIN.x + 11, LNAV_WIN.x + 15
    writeLabel(t, lnavLblL, 8, "HDG")
    if state.heading ~= nil then
        writeAt(t, lnavValL, 8, string.format("%5s", string.format("%03.0f", state.heading)))
    else
        writeAt(t, lnavValL, 8, "  --")
    end
    writeLabel(t, lnavLblR, 8, "YAW")
    if state.yawRate ~= nil then
        writeAt(t, lnavValR, 8, string.format("%+5.1f", state.yawRate))
    else
        writeAt(t, lnavValR, 8, "  --")
    end
    writeLabel(t, VNAV_WIN.x + 1, 8, "AGL")
    if state.agl ~= nil then
        writeAt(t, VNAV_WIN.x + 5, 8, string.format("%5.1f m", state.agl))
    else
        writeAt(t, VNAV_WIN.x + 5, 8, "   --")
    end
    writeLabel(t, VNAV_WIN.x + 14, 8, "VOL")
    if state.balloonCapacity == nil then
        writeAt(t, VNAV_WIN.x + 18, 8, "  --")
    else
        writeAt(t, VNAV_WIN.x + 18, 8, string.format("%5.0f", state.balloonCapacity))
    end

    -- Row 9: left/right props under LNAV, vertical prop under VNAV
    drawSplitRow(t, 9, {MODE_SPLIT})
    writeLabel(t, lnavLblL, 9, "L")
    writeAt(t, lnavValL, 9, string.format("%5d", math.floor((state.propeller1Rpm or 0) + 0.5)))
    writeLabel(t, lnavLblR, 9, "R")
    writeAt(t, lnavValR, 9, string.format("%5d", math.floor((state.propeller2Rpm or 0) + 0.5)))
    if state.speedReduced then
        writeWin(t, LNAV_WIN, 9, "CUT", COL_MANUAL, "right")
    end
    writeLabel(t, VNAV_WIN.x + 1, 9, "VP")
    writeAt(t, VNAV_WIN.x + 5, 9, string.format("%5d", math.floor((state.verticalPropRpm or 0) + 0.5)))
    writeLabel(t, VNAV_WIN.x + 14, 9, "V/S")
    if state.verticalSpeed ~= nil then
        writeAt(t, VNAV_WIN.x + 18, 9, string.format("%+5.1f", state.verticalSpeed))
    else
        writeAt(t, VNAV_WIN.x + 18, 9, "  --")
    end

    -- Row 10: close the detail panes
    drawBorderLine(t, 10, BORDER_DETAIL_ATT)

    -- Row 11: ATT mode + pitch + stabilizer current/target
    drawRow(t, 11)
    local att = ATT_MODE[state.attMode] or ATT_MODE.off
    writeLabel(t, 3, 11, "ATT")
    writeAt(t, 7, 11, att.label, att.color)
    writeLabel(t, 16, 11, "PITCH")
    local pitchColor = state.attFault and colors.red or COL_VALUE
    if state.pitch ~= nil then
        writeAt(t, 22, 11, string.format("%+5.1f", state.pitch), pitchColor)
    else
        writeAt(t, 22, 11, "  --", pitchColor)
    end
    writeLabel(t, 29, 11, "STAB")
    if state.attMode == "none" then
        writePair(t, 34, 11, fmtPairNum("%5.1f", nil), fmtPairNum("%5.1f", nil), COL_MANUAL, COL_MANUAL)
    else
        writePair(t, 34, 11,
            fmtPairNum("%5.1f", state.stabAngle),
            fmtPairNum("%5.1f", state.stabTarget or 0))
    end

    drawBorderLine(t, 12, BORDER_BTM)
    for y = 13, H do
        drawRow(t, y)
    end
end
