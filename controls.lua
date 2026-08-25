-- Collect independent mainThread peripheral writes and run them in one
-- server tick. Sequential setters each yield until the next tick; batched
-- ones share a tick. A single write is called directly — wrapping that in
-- parallel is just overhead.
-- https://solastrius.github.io/CreateAvionics/guide/mainthread.html
WriteBatch = {}

function WriteBatch:new()
    local t = setmetatable({}, { __index = WriteBatch })
    t.fns = {}
    return t
end

function WriteBatch:add(fn)
    if fn ~= nil then
        self.fns[#self.fns + 1] = fn
    end
end

-- Run now when no batch is in play (constructors, debug). Otherwise enqueue.
function WriteBatch.defer(batch, fn)
    if fn == nil then
        return
    end
    if batch ~= nil then
        batch:add(fn)
    else
        fn()
    end
end

function WriteBatch:flush()
    local n = #self.fns
    if n == 0 then
        return
    end
    if n == 1 then
        self.fns[1]()
    else
        parallel.waitForAll(table.unpack(self.fns))
    end
    self.fns = {}
end

-- OnChangeOutput wraps a setter function and only calls it when the value changes.
-- Useful for physical outputs (levers, indicators) that make noise or flicker on
-- repeated identical writes.
-- An optional tolerance can be provided to suppress changes smaller than that amount.
OnChangeOutput = {}

function OnChangeOutput:new(setFn, tolerance, round)
    local t = setmetatable({}, { __index = OnChangeOutput })
    t.setFn = setFn
    t.tolerance = tolerance or 0
    t.round = round or false
    t.lastValue = nil
    return t
end

function OnChangeOutput:set(value)
    local v = self.round and math.floor(value + 0.5) or value
    if self.lastValue == nil or math.abs(v - self.lastValue) > self.tolerance then
        self.setFn(v)
        self.lastValue = v
    end
end

function OnChangeOutput:invalidate()
    self.lastValue = nil
end
