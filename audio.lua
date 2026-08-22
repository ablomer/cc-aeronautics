-- ShipAudio plays short noteblock cues for flight-system events.
-- Uses speaker.playNote (instrument, volume, pitch). Never blocks the
-- control loop: a missing speaker or a failed note is ignored.

ShipAudio = {}

-- Seconds between repeating alerts while the condition stays true.
-- Mode-change cues fire immediately; the interval only spaces the repeats.
ShipAudio.TERRAIN_INTERVAL = 1.5
ShipAudio.FAULT_INTERVAL   = 2.0

-- Each cue is a list of {instrument, volume, pitch} notes played in one tick.
-- Pitch is semitones 0-24 (0/12/24 = F#, 6/18 = C). Volume is 0.0-3.0.
-- At most 8 notes can play in a single tick.
local CUES = {
    ready = {
        { "chime", 0.8, 6 },
        { "chime", 0.8, 10 },
        { "chime", 0.8, 13 },
    },
    lnav_stop = {
        { "bass", 1.0, 6 },
    },
    lnav_hold = {
        { "harp", 0.8, 12 },
    },
    lnav_nav = {
        { "bit", 1.0, 12 },
        { "bit", 1.0, 16 },
    },
    vnav_hold = {
        { "harp", 0.8, 15 },
    },
    vnav_land = {
        { "bass", 1.0, 8 },
        { "bass", 1.0, 4 },
    },
    vnav_flare = {
        { "flute", 1.2, 18 },
    },
    vnav_landed = {
        { "bell",  1.2, 12 },
        { "chime", 1.0, 18 },
        { "chime", 1.0, 24 },
    },
    terrain = {
        { "cow_bell", 1.5, 18 },
    },
    fault = {
        { "snare", 1.5, 12 },
        { "hat",   1.2, 18 },
    },
    nav_acquire = {
        { "pling", 0.8, 15 },
    },
    nav_lost = {
        { "pling", 0.8, 8 },
    },
}

local LNAV_CUES = {
    stop = "lnav_stop",
    hold = "lnav_hold",
    nav  = "lnav_nav",
}

local VNAV_CUES = {
    hold    = "vnav_hold",
    land    = "vnav_land",
    flare   = "vnav_flare",
    landed  = "vnav_landed",
    terrain = "terrain",
}

function ShipAudio:new(speakers)
    local t = setmetatable({}, { __index = ShipAudio })
    t.speakers = speakers or {}
    t.lastLnav = nil
    t.lastVnav = nil
    t.lastNavActive = nil
    t.lastFault = false
    t.lastTerrainWarn = nil
    t.lastFaultWarn = nil
    t:playCue("ready")
    return t
end

function ShipAudio:playCue(name)
    local notes = CUES[name]
    if notes == nil then
        return
    end
    for _, speaker in ipairs(self.speakers) do
        if speaker ~= nil then
            for _, note in ipairs(notes) do
                -- pcall so a detached speaker or a rejected note cannot stall flight.
                pcall(speaker.playNote, note[1], note[2], note[3])
            end
        end
    end
end


-- state uses the same snapshot as FlightDisplay:update:
--   lnavMode, vnavMode, navActive, altitudeFault
function ShipAudio:update(state)
    local now = os.clock()

    if self.lastLnav ~= nil and state.lnavMode ~= self.lastLnav then
        local cue = LNAV_CUES[state.lnavMode]
        if cue then
            self:playCue(cue)
        end
    end
    self.lastLnav = state.lnavMode

    if self.lastVnav ~= nil and state.vnavMode ~= self.lastVnav then
        local cue = VNAV_CUES[state.vnavMode]
        if cue then
            self:playCue(cue)
        end
        if state.vnavMode == "terrain" then
            self.lastTerrainWarn = now
        end
    end
    self.lastVnav = state.vnavMode

    if self.lastNavActive ~= nil and state.navActive ~= self.lastNavActive then
        self:playCue(state.navActive and "nav_acquire" or "nav_lost")
    end
    self.lastNavActive = state.navActive

    if state.vnavMode == "terrain" then
        if self.lastTerrainWarn == nil or now - self.lastTerrainWarn >= ShipAudio.TERRAIN_INTERVAL then
            self:playCue("terrain")
            self.lastTerrainWarn = now
        end
    end

    if state.altitudeFault then
        local justSet = not self.lastFault
        local due = self.lastFaultWarn == nil or now - self.lastFaultWarn >= ShipAudio.FAULT_INTERVAL
        if justSet or due then
            self:playCue("fault")
            self.lastFaultWarn = now
        end
    end
    self.lastFault = state.altitudeFault == true
end
