-- ShipAudio plays DFPWM callouts for flight-system events.
-- speaker.playAudio is never waited on: remaining PCM is pushed on
-- speaker_audio_empty. A missing speaker, clip, or rejected play is ignored.
-- Speakers are not mixed into the Create WriteBatch — playAudio's boolean
-- must be read, and several speakers still start together via parallel.
--
-- Convert audio/*.mp3 with convert-audio.sh (FFmpeg 5.1+ DFPWM1a, 48 kHz mono).
-- https://tweaked.cc/peripheral/speaker.html#v:playAudio
-- https://tweaked.cc/guide/speaker_audio.html

require("config")

local hasDfpwm, dfpwm = pcall(require, "cc.audio.dfpwm")
if not hasDfpwm then
    dfpwm = nil
end

ShipAudio = {}

-- Seconds between repeating alerts while the condition stays true.
-- Mode-change cues fire immediately; the interval only spaces the repeats.
ShipAudio.FAULT_INTERVAL = 2.0
-- Seconds after [ DELAY ] before the test chime plays.
ShipAudio.CHIME_DELAY = 8.0

-- speaker.playAudio plays 48 kHz PCM and accepts at most 128×1024 samples
-- (~2.7 s) in one call. First try that; if the peripheral rejects it,
-- fall back to 16 Ki-sample chunks continued on speaker_audio_empty.
ShipAudio.SAMPLE_RATE   = 48000
ShipAudio.MAX_SAMPLES   = 128 * 1024
ShipAudio.CHUNK_SAMPLES = 16 * 1024

-- speaker.playAudio volume is 0.0-3.0. SHIP.AUDIO.volume is the source.
local function speakerVolume()
    local v = SHIP.AUDIO and SHIP.AUDIO.volume
    if type(v) ~= "number" or v ~= v then
        v = 3.0
    end
    if v < 0 then
        return 0
    end
    if v > 3 then
        return 3
    end
    return v
end

-- Cue name -> audio/<stem>.dfpwm (from convert-audio.sh). A table of stems
-- is tried in order so ready can stay named system_ready.
local CLIPS = {
    ready    = { "system_ready", "ready" },
    stop     = "stop",
    speed    = "speed",
    altitude = "altitude",
    land     = "land",
    flare    = "flare",
    terrain  = "terrain",
    landed   = "landed",
    fault    = "fault",
    nav      = "nav",
    wheel    = "wheel",
    pattern  = "pattern",
    direct   = "direct",
    arrived  = "arrived",
    attitude = "attitude",
    chime    = "chime",
}

-- Higher number preempts. Repeating alerts sit at the top so they can
-- cut a mode callout; equal priority replaces so a new mode is heard.
local PRIORITY = {
    terrain  = 3,
    fault    = 3,
    attitude = 3,
    flare    = 2,
    land     = 2,
    landed   = 2,
    nav      = 1,
    wheel    = 1,
    pattern  = 1,
    direct   = 1,
    arrived  = 1,
    speed    = 0,
    stop     = 0,
    altitude = 0,
    ready    = 0,
    chime    = 4,
}

local LNAV_CUES = {
    stop = "stop",
    hold = "speed",
}

local VNAV_CUES = {
    hold    = "altitude",
    land    = "land",
    flare   = "flare",
    landed  = "landed",
    terrain = "terrain",
}

local function programDir()
    local prog = nil
    if type(shell) == "table" and type(shell.getRunningProgram) == "function" then
        prog = shell.getRunningProgram()
    end
    if type(prog) == "string" and prog ~= "" then
        return fs.getDir(prog)
    end
    return ""
end

local function clipStems(cue)
    local names = CLIPS[cue]
    if names == nil then
        return { cue }
    end
    if type(names) == "string" then
        return { names }
    end
    return names
end

local function runParallel(fns)
    if #fns == 0 then
        return
    end
    if #fns == 1 then
        fns[1]()
        return
    end
    parallel.waitForAll(table.unpack(fns))
end

local function copySlice(pcm, first, last)
    local chunk = {}
    local k = 1
    for i = first, last do
        local v = pcm[i]
        if type(v) == "number" then
            if v >= 0 then
                v = math.floor(v + 0.5)
            else
                v = math.ceil(v - 0.5)
            end
            if v > 127 then
                v = 127
            elseif v < -128 then
                v = -128
            end
            chunk[k] = v
            k = k + 1
        end
    end
    return chunk
end

function ShipAudio:new(speakers)
    local t = setmetatable({}, { __index = ShipAudio })
    t.speakers = speakers or {}
    t.encoded = {}
    t.lastLnav = nil
    t.lastVnav = nil
    t.lastSteer = nil
    t.lastPattern = nil
    t.lastArrived = nil
    t.lastFault = false
    t.lastAttFault = false
    t.lastFaultWarn = nil
    t.lastTerrainWarn = nil
    t.lastAttWarn = nil
    t.chimeAt = nil
    t.busyUntil = nil
    t.playPriority = 0
    t.playingCue = nil
    t.lastError = nil
    t.streamPcm = nil
    t.streamVolume = speakerVolume()
    t.streamPos = {}
    t.speakerByName = {}
    for _, spk in ipairs(t.speakers) do
        if spk ~= nil then
            local ok, name = pcall(peripheral.getName, spk)
            if ok and type(name) == "string" then
                t.speakerByName[name] = spk
            end
        end
    end
    t:loadEncoded()
    t:playCue("ready")
    return t
end

function ShipAudio:loadEncoded()
    self.encoded = {}
    if dfpwm == nil or fs == nil then
        return
    end
    local dir = fs.combine(programDir(), "audio")
    local ok, files = pcall(function()
        if not fs.exists(dir) then
            return nil
        end
        return fs.list(dir)
    end)
    if not ok or type(files) ~= "table" then
        return
    end
    for i = 1, #files do
        local file = files[i]
        local stem = type(file) == "string" and file:match("^(.*)%.[Dd][Ff][Pp][Ww][Mm]$")
        if stem then
            local handle = fs.open(fs.combine(dir, file), "rb")
            if handle ~= nil then
                local data = handle.readAll()
                handle.close()
                if type(data) == "string" and #data > 0 then
                    self.encoded[stem] = data
                end
            end
        end
    end
end

function ShipAudio:encodedStem(cue)
    local names = clipStems(cue)
    for i = 1, #names do
        if self.encoded[names[i]] ~= nil then
            return names[i]
        end
    end
    return nil
end

function ShipAudio:isBusy(now)
    now = now or os.clock()
    return self.busyUntil ~= nil and now < self.busyUntil
end

function ShipAudio:isPlaying(name, now)
    return name ~= nil and self.playingCue == name and self:isBusy(now)
end

-- Arm or cancel the delayed test chime. Returns true when a countdown
-- is now running.
function ShipAudio:armChime(seconds)
    if self.chimeAt ~= nil then
        self.chimeAt = nil
        return false
    end
    self.chimeAt = os.clock() + (seconds or ShipAudio.CHIME_DELAY)
    return true
end

function ShipAudio:cancelChimeDelay()
    self.chimeAt = nil
end

function ShipAudio:chimeDelayLeft(now)
    now = now or os.clock()
    if self.chimeAt == nil then
        return nil
    end
    local left = self.chimeAt - now
    if left < 0 then
        return 0
    end
    return left
end

function ShipAudio:pushChunk(spk, pcm, first, last)
    local chunk = copySlice(pcm, first, last)
    if #chunk == 0 then
        return false
    end
    local ok, ret = pcall(spk.playAudio, chunk, self.streamVolume)
    if not ok then
        self.lastError = tostring(ret)
        return false
    end
    if ret ~= true then
        self.lastError = "playAudio full"
        return false
    end
    return true
end

function ShipAudio:stopAll()
    for _, spk in ipairs(self.speakers) do
        if spk ~= nil then
            pcall(spk.stop)
        end
    end
    self.streamPcm = nil
    self.streamPos = {}
end

-- Push the next PCM slice to one speaker. Returns true if accepted.
function ShipAudio:pushNext(name)
    local spk = self.speakerByName[name]
    local pcm = self.streamPcm
    local pos = self.streamPos[name]
    if spk == nil or pcm == nil or pos == nil or pos > #pcm then
        return false
    end
    local last = pos + ShipAudio.CHUNK_SAMPLES - 1
    if last > #pcm then
        last = #pcm
    end
    if not self:pushChunk(spk, pcm, pos, last) then
        return false
    end
    self.streamPos[name] = last + 1
    self.lastError = nil
    return true
end

function ShipAudio:onSpeakerEmpty(name)
    if type(name) == "string" and self:pushNext(name) then
        return
    end
    -- Event name did not match peripheral.getName; continue any speaker
    -- that still has samples so chunked playback cannot stall.
    for n, _ in pairs(self.speakerByName) do
        if self:pushNext(n) then
            return
        end
    end
end

-- Start the same clip on every speaker. Tries one playAudio of the whole
-- buffer first (up to MAX_SAMPLES). If that is rejected, streams chunks.
function ShipAudio:startStream(pcm, now)
    self:stopAll()
    self.streamPcm = pcm
    self.streamVolume = speakerVolume()
    local names = {}
    for name, _ in pairs(self.speakerByName) do
        names[#names + 1] = name
        self.streamPos[name] = 1
    end
    if #names == 0 then
        self.lastError = "no speaker names"
        return false
    end

    local accepted = false
    local whole = #pcm <= ShipAudio.MAX_SAMPLES
    local fns = {}
    for i = 1, #names do
        local name = names[i]
        fns[i] = function()
            if whole then
                local spk = self.speakerByName[name]
                if spk ~= nil and self:pushChunk(spk, pcm, 1, #pcm) then
                    self.streamPos[name] = #pcm + 1
                    accepted = true
                end
            elseif self:pushNext(name) then
                accepted = true
            end
        end
    end
    runParallel(fns)

    -- Whole-buffer play failed: stream smaller chunks instead.
    if not accepted and whole then
        self.lastError = nil
        for name, _ in pairs(self.speakerByName) do
            self.streamPos[name] = 1
        end
        fns = {}
        for i = 1, #names do
            local name = names[i]
            fns[i] = function()
                if self:pushNext(name) then
                    accepted = true
                end
            end
        end
        runParallel(fns)
    end

    if not accepted then
        if self.lastError == nil then
            self.lastError = "playAudio rejected"
        end
        self:stopAll()
        return false
    end
    self.busyUntil = now + (#pcm / ShipAudio.SAMPLE_RATE)
    return true
end

-- Decode a short DFPWM clip and start playback. True only if at least
-- one speaker accepted a playAudio buffer.
function ShipAudio:playSample(stem, priority, now)
    if dfpwm == nil then
        self.lastError = "no cc.audio.dfpwm"
        return false
    end
    if #self.speakers == 0 then
        self.lastError = "no speakers"
        return false
    end
    local data = self.encoded[stem]
    if data == nil then
        self.lastError = "missing " .. tostring(stem) .. ".dfpwm"
        return false
    end
    if self:isBusy(now) and (self.playPriority or 0) > priority then
        return false
    end
    local ok, pcm = pcall(dfpwm.decode, data)
    if not ok then
        self.lastError = "decode: " .. tostring(pcm)
        return false
    end
    if type(pcm) ~= "table" or #pcm == 0 then
        self.lastError = "decode empty"
        return false
    end
    if not self:startStream(pcm, now) then
        return false
    end
    self.playPriority = priority
    self.playingCue = stem
    return true
end

function ShipAudio:playCue(name, now, _batch)
    now = now or os.clock()
    local stem = self:encodedStem(name)
    if stem == nil then
        self.lastError = "missing " .. tostring(name) .. ".dfpwm"
        return false
    end
    if not self:playSample(stem, PRIORITY[name] or 0, now) then
        return false
    end
    self.playingCue = name
    return true
end

local function due(last, now)
    return last == nil or now - last >= ShipAudio.FAULT_INTERVAL
end

-- state uses the same snapshot as FlightDisplay:update, plus arrived:
--   lnavMode, vnavMode, altitudeFault, attFault,
--   steerSource, navPattern, arrived
function ShipAudio:update(state, _batch)
    local now = os.clock()

    if self.chimeAt ~= nil and now >= self.chimeAt then
        self.chimeAt = nil
        self:playCue("chime", now)
    end

    if self.lastLnav ~= nil and state.lnavMode ~= self.lastLnav then
        local cue = LNAV_CUES[state.lnavMode]
        if cue then
            self:playCue(cue, now)
        end
    end
    self.lastLnav = state.lnavMode

    if self.lastVnav ~= nil and state.vnavMode ~= self.lastVnav then
        local cue = VNAV_CUES[state.vnavMode]
        if cue then
            self:playCue(cue, now)
        end
        if state.vnavMode == "terrain" then
            self.lastTerrainWarn = now
        end
    end
    self.lastVnav = state.vnavMode

    local src = state.steerSource
    if self.lastSteer ~= nil and src ~= self.lastSteer then
        if src == "NAV" then
            self:playCue("nav", now)
        else
            self:playCue("wheel", now)
        end
    end
    -- Pattern is NAV-only. Dropping the compass also clears PTN; that
    -- is "Wheel", not a second "Direct".
    if src == "NAV" and self.lastPattern ~= nil and state.navPattern ~= self.lastPattern then
        if state.navPattern then
            self:playCue("pattern", now)
        else
            self:playCue("direct", now)
        end
    end
    self.lastSteer = src
    self.lastPattern = state.navPattern == true

    if self.lastArrived ~= nil and state.arrived and not self.lastArrived then
        self:playCue("arrived", now)
    end
    self.lastArrived = state.arrived == true

    local anyFault = state.altitudeFault == true
    if anyFault then
        local justSet = not self.lastFault
        if justSet or (due(self.lastFaultWarn, now) and not self:isBusy(now)) then
            self:playCue("fault", now)
            self.lastFaultWarn = now
        end
    end
    self.lastFault = anyFault

    local attFault = state.attFault == true
    if attFault then
        local justSet = not self.lastAttFault
        if justSet or (due(self.lastAttWarn, now) and not self:isBusy(now)) then
            self:playCue("attitude", now)
            self.lastAttWarn = now
        end
    end
    self.lastAttFault = attFault

    if state.vnavMode == "terrain" then
        if due(self.lastTerrainWarn, now) and not self:isBusy(now) then
            self:playCue("terrain", now)
            self.lastTerrainWarn = now
        end
    end
end
