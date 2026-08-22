-- ShipAudio plays short noteblock cues for flight-system events.
-- Uses speaker.playNote (instrument, volume, pitch). Never blocks the
-- control loop: a missing speaker or a failed note is ignored.

require("config")

ShipAudio = {}

-- Seconds between repeating alerts while the condition stays true.
-- Mode-change cues fire immediately; the interval only spaces the repeats.
ShipAudio.FAULT_INTERVAL = 2.0

-- Proximity: beep rate and pitch rise as AGL closes on
-- SHIP.VNAV.touchdownAgl. Used during flare and while TERRAIN (ground
-- protection) is commanding a climb. Starts at first optical contact
-- (same band as VNav.OPTICAL_RANGE) and stops once VNAV latches landed.
ShipAudio.PROXIMITY_START         = 15.0
ShipAudio.PROXIMITY_INTERVAL_FAR  = 1.0
ShipAudio.PROXIMITY_INTERVAL_NEAR = 0.12
ShipAudio.PROXIMITY_PITCH_FAR     = 12
ShipAudio.PROXIMITY_PITCH_NEAR    = 24
ShipAudio.PROXIMITY_VOLUME        = 1.0

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
    t.lastFaultWarn = nil
    t.lastProximityBeep = nil
    t:playCue("ready")
    return t
end

function ShipAudio:playNote(instrument, volume, pitch)
    for _, speaker in ipairs(self.speakers) do
        if speaker ~= nil then
            -- pcall so a detached speaker or a rejected note cannot stall flight.
            pcall(speaker.playNote, instrument, volume, pitch)
        end
    end
end

function ShipAudio:playCue(name)
    local notes = CUES[name]
    if notes == nil then
        return
    end
    for _, note in ipairs(notes) do
        self:playNote(note[1], note[2], note[3])
    end
end


-- t is 1 at first contact and 0 at/below touchdown. Interval and pitch
-- both lerp along that (faster + higher as the hull settles).
function ShipAudio:proximityBeep(agl, now)
    local settle = SHIP.VNAV.touchdownAgl
    local span = ShipAudio.PROXIMITY_START - settle
    local t = 0
    if span > 0 then
        t = (agl - settle) / span
        if t < 0 then t = 0 elseif t > 1 then t = 1 end
    end
    local interval = ShipAudio.PROXIMITY_INTERVAL_NEAR
        + t * (ShipAudio.PROXIMITY_INTERVAL_FAR - ShipAudio.PROXIMITY_INTERVAL_NEAR)
    if self.lastProximityBeep ~= nil and now - self.lastProximityBeep < interval then
        return
    end
    local pitch = ShipAudio.PROXIMITY_PITCH_NEAR
        + t * (ShipAudio.PROXIMITY_PITCH_FAR - ShipAudio.PROXIMITY_PITCH_NEAR)
    self:playNote("pling", ShipAudio.PROXIMITY_VOLUME, pitch)
    self.lastProximityBeep = now
end

-- state uses the same snapshot as FlightDisplay:update:
--   lnavMode, vnavMode, navActive, altitudeFault, landing, agl
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
    end
    self.lastVnav = state.vnavMode

    if self.lastNavActive ~= nil and state.navActive ~= self.lastNavActive then
        self:playCue(state.navActive and "nav_acquire" or "nav_lost")
    end
    self.lastNavActive = state.navActive

    if state.altitudeFault then
        local justSet = not self.lastFault
        local due = self.lastFaultWarn == nil or now - self.lastFaultWarn >= ShipAudio.FAULT_INTERVAL
        if justSet or due then
            self:playCue("fault")
            self.lastFaultWarn = now
        end
    end
    self.lastFault = state.altitudeFault == true

    -- Flare or TERRAIN climb, with optical AGL. Descent before first
    -- contact and the landed latch stay silent so this does not fight
    -- those cues. TERRAIN still gets a one-shot cowbell on engage.
    local wantProximity = state.agl ~= nil and (
        state.vnavMode == "terrain"
        or (state.landing and state.vnavMode ~= "landed")
    )
    if wantProximity then
        self:proximityBeep(state.agl, now)
    else
        self.lastProximityBeep = nil
    end

end

