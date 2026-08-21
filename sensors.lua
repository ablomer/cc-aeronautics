-- OpticalSensorBank aggregates one or more downward-facing optical distance
-- sensors and reports the single minimum (most conservative) reading each
-- tick. This is telemetry only right now: nothing in the control loop acts
-- on its output yet. That comes later, once ground-protection authority is
-- added on top of it.
--
-- Confirmed in-game: getDistance() reports a real 0-15 metre reading when
-- something is within range (15 itself is a genuine close reading, not a
-- sentinel), and reports exactly 15.5 when nothing is within range at all.
-- 15.5 is therefore treated as "clear" -- a healthy, no-hazard reading --
-- and normalized down to MAX_DISTANCE (15) so it participates correctly in
-- the cross-sensor minimum below. Anything else outside 0-15 (nil, NaN,
-- error, or some other out-of-range number) is not a recognized sentinel
-- and is treated as a sensor fault rather than guessed at.
OpticalSensorBank = {}

OpticalSensorBank.MIN_DISTANCE = 0
OpticalSensorBank.MAX_DISTANCE = 15
OpticalSensorBank.NO_TARGET_VALUE = 15.5
local NO_TARGET_EPSILON = 0.01  -- tolerance for float comparison against 15.5

local function isValidDistance(d)
    return type(d) == "number"
        and d == d -- rejects NaN
        and d >= OpticalSensorBank.MIN_DISTANCE
        and d <= OpticalSensorBank.MAX_DISTANCE
end

local function isNoTarget(d)
    return type(d) == "number"
        and d == d -- rejects NaN
        and math.abs(d - OpticalSensorBank.NO_TARGET_VALUE) < NO_TARGET_EPSILON
end

-- sensorNames: array of peripheral names, e.g. { "optical_sensor_0" }.
-- Missing peripherals (wrap fails) are kept in the list as unhealthy rather
-- than silently dropped, so a detached sensor shows up in telemetry instead
-- of just quietly reducing the sensor count.
function OpticalSensorBank:new(sensorNames)
    local t = setmetatable({}, { __index = OpticalSensorBank })
    t.sensors = {}
    for _, name in ipairs(sensorNames) do
        table.insert(t.sensors, { name = name, peripheral = peripheral.wrap(name) })
    end
    t.lastMinDistance = OpticalSensorBank.MAX_DISTANCE
    t.lastMinSensor = nil
    t.lastReadings = {}
    t.healthyCount = 0
    t.faultCount = 0
    t.fault = false
    return t
end

-- Reads every configured sensor and returns (minDistance, minSensorName).
-- Call once per tick; this is stateful telemetry (lastReadings, counts)
-- like the other read()-style controllers in this codebase.
--
-- A sensor is excluded from the minimum if its peripheral is missing, its
-- getDistance() call errors (protected by pcall so one bad peripheral can't
-- take down the control loop), or its value is neither a valid 0-15 reading
-- nor the confirmed 15.5 "clear" sentinel. A "clear" reading is healthy and
-- is normalized to MAX_DISTANCE for the purposes of the cross-sensor minimum.
--
-- If every configured sensor is unhealthy this tick, self.fault is set and
-- the bank reports OpticalSensorBank.MIN_DISTANCE (0) rather than "clear" --
-- assume the worst when there is no trustworthy data at all.
function OpticalSensorBank:read()
    local readings = {}
    local minDistance = nil
    local minSensor = nil
    local healthy = 0
    local faulted = 0

    for _, sensor in ipairs(self.sensors) do
        local distance = nil
        local ok = false

        if sensor.peripheral then
            local success, value = pcall(sensor.peripheral.getDistance)
            if success then
                if isValidDistance(value) then
                    distance = value
                    ok = true
                elseif isNoTarget(value) then
                    distance = OpticalSensorBank.MAX_DISTANCE
                    ok = true
                end
            end
        end

        if ok then
            healthy = healthy + 1
            if minDistance == nil or distance < minDistance then
                minDistance = distance
                minSensor = sensor.name
            end
        else
            faulted = faulted + 1
        end

        table.insert(readings, { name = sensor.name, distance = distance, healthy = ok })
    end

    self.fault = healthy == 0
    if self.fault then
        minDistance = OpticalSensorBank.MIN_DISTANCE
        minSensor = nil
    end

    self.lastReadings = readings
    self.lastMinDistance = minDistance
    self.lastMinSensor = minSensor
    self.healthyCount = healthy
    self.faultCount = faulted

    return minDistance, minSensor
end
