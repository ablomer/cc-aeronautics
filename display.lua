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

local BORDER_TOP = "\xc9" .. string.rep("\xcd", W - 2) .. "\xbb"
local BORDER_MID = "\xcc" .. string.rep("\xcd", W - 2) .. "\xb9"
local BORDER_BTM = "\xc8" .. string.rep("\xcd", W - 2) .. "\xbc"
local BORDER_ROW = "\xba" .. string.rep(" ",    W - 2) .. "\xba"

-- Button hit regions (x1, x2, y) — updated each render when in hold mode
local BTN_DEC = { x1 = 26, x2 = 30, y = 8 }  -- [ - ]
local BTN_INC = { x1 = 32, x2 = 36, y = 8 }  -- [ + ]

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

-- Returns "inc", "dec", or nil depending on which button was clicked.
function FlightDisplay:hitTest(x, y)
    if x >= BTN_DEC.x1 and x <= BTN_DEC.x2 and y == BTN_DEC.y then
        return "dec"
    end
    if x >= BTN_INC.x1 and x <= BTN_INC.x2 and y == BTN_INC.y then
        return "inc"
    end
    return nil
end

function FlightDisplay:update(state)
    -- state = {
    --   velocity: number (m/s)
    --   throttle: number (0-15)
    --   holdMode: boolean
    --   targetVelocity: number|nil (only relevant in hold mode)
    --   steeringAngle: number (-180 to 180, degrees)
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

    -- Row 5: mode
    drawRow(t, 5)
    writeLabel(t, 3, 5, "MODE")
    if state.holdMode then
        writeAt(t, 14, 5, "[ VELOCITY HOLD ]", COL_HOLD)
    else
        writeAt(t, 14, 5, "[ MANUAL ]       ", COL_MANUAL)
    end

    -- Row 6: blank
    drawRow(t, 6)

    -- Row 7: velocity
    drawRow(t, 7)
    writeLabel(t, 3, 7, "VELOCITY")
    writeAt(t, 14, 7, string.format("%-10s", string.format("%.2f m/s", state.velocity)))

    -- Row 8: target velocity with +/- buttons (hold mode only) or blank
    drawRow(t, 8)
    if state.holdMode and state.targetVelocity ~= nil then
        writeLabel(t, 3, 8, "TARGET  ")
        writeAt(t, 14, 8, string.format("%-10s", string.format("%.2f m/s", state.targetVelocity)), COL_HOLD)
        drawButton(t, BTN_DEC.x1, 8, "[ - ]")
        drawButton(t, BTN_INC.x1, 8, "[ + ]")
    end

    -- Row 9: throttle
    drawRow(t, 9)
    writeLabel(t, 3, 9, "THROTTLE")
    writeAt(t, 14, 9, string.format("%-10s", string.format("%d / 15", state.throttle)))

    -- Row 10: steering angle (debug)
    drawRow(t, 10)
    writeLabel(t, 3, 10, "STEERING")
    writeAt(t, 14, 10, string.format("%-10s", string.format("%.1f deg", state.steeringAngle or 0)))

    -- Row 11: blank
    drawRow(t, 11)

    -- Row 12: bottom border
    drawBorderLine(t, 12, BORDER_BTM)
end
