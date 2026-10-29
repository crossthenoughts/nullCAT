-- SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
-- SPDX-License-Identifier: GPL-3.0-or-later
--
-- nullCAT telemetry for BeamNG.drive (game 0.32 and later): a UDP protocol
-- that sends nullCAT's named-channel line (NULLCATY, nullCAT's
-- docs/PROTOCOL.md) straight to the rig, for its haptics and belts. Motion
-- software (SimHub, SimTools, FlyPT Mover, ...) keeps reading BeamNG's own
-- MotionSim protocol; this is only what nullCAT plays. It says direct=only
-- with the car's name (it is a direct source and cannot switch), so
-- nullCAT takes the channels from it and sets aside another sender's for
-- the game (the SimHub plugin can stay on). Only the car you are in
-- sends.
--
-- Where it sends and how often: the settings protocols_nullcat_zerowerks_
-- address, _port and _maxUpdateRate when the game has them, else the
-- defaults below. A rate of 60 or 100 reads once per rendered frame (cheap,
-- and never faster than the frame rate); 200 reads on the physics step (it
-- costs frame rate). Reload the vehicle (Ctrl+R) after changing a setting
-- or this file.

local M = {}

local NAME     = "nullcat_zerowerks"
local ADDRESS  = "127.0.0.1"     -- where nullCAT listens: 127.0.0.1 with nullCAT on this PC, the Pi on a Pi rig (192.168.50.1, as the SimHub plugin's host)
local PORT     = 4444            -- nullCAT's telemetry port
local RATE     = 100             -- 60, 100 or 200 a second
local SEND_HEIGHTS = true        -- the road height under each tyre (four ray casts a send)
local DEBUG_KEYS   = false       -- extra keys starting with "d" (nullCAT skips them, its recorder keeps them): the raw g three ways, the slip energy, the fill step
local DUMP_ENGINE  = true        -- what the game says about the engine and the build, once per car, to its log (lines "nullCAT: ...")

local LINE_LEN       = 1400      -- the datagram: the line, then spaces (nullCAT trims them)
local SLOW_EVERY_SEC = 0.1       -- tyres, engine limit, car size
local ID_EVERY_SEC   = 1.0       -- the car's name
local LIMITER_HOLD_SEC = 0.25   -- the limiter held on between its cuts
local GAME           = "BeamNG.drive"
local RAD2DEG        = 57.29577951308232

local ffi = require("ffi")

-- BeamNG's ground model names onto nullCAT's surface classes (0 tarmac,
-- 1 bumpy tarmac, 2 kerb, 3 gravel, 4 grass, 5 dirt, 6 cobbles, 7 snow,
-- 8 ice, 9 sand, 10 mud, 11 standing water). Anything else reads as
-- tarmac and is named once in the game's log.
local SURFACE = {
  ASPHALT = 0, ASPHALT_WET = 0, SLIPPERY = 0, METAL = 0, WOOD = 0, SPIKE_STRIP = 0,
  ROCK = 1, RUMBLE_STRIP = 2, GRAVEL = 3, GRASS = 4, FOLIAGE = 4, FOLIAGE_THIN = 4,
  DIRT = 5, DIRT_DUSTY = 5, SANDY_ROAD = 5, COBBLESTONE = 6, SNOW = 7, ICE = 8,
  SAND = 9, MUD = 10, WATER = 11,
}
local WET = { ASPHALT_WET = true, WATER = true }

local CORNERS = { "FL", "FR", "RL", "RR" }

local function say(level, msg)
  if log then log(level, "nullCAT", msg) end
end

local function setting(key, default)
  if not (settings and settings.getValue) then return default end
  local v = settings.getValue("protocols_" .. NAME .. "_" .. key)
  if v == nil or v == "" then return default end
  return v
end

local function rate()
  local r = tonumber(setting("maxUpdateRate", RATE)) or RATE
  if r >= 150 then return 200 elseif r >= 80 then return 100 end
  return 60
end

-- ---- what the car is: its wheels, the surfaces, its name and size

local corner = {}        -- "FL".. -> the wheel
local matClass, matWet = {}, {}
local unknownSurface = {}
local carName, carId = "", ""
local wheelbase, trackWidth = nil, nil
local engine = nil

local function findWheels()
  corner = {}
  if not (wheels and wheels.wheels) then return end
  for _, w in pairs(wheels.wheels) do
    local n = w and w.name and string.upper(tostring(w.name))
    for _, c in ipairs(CORNERS) do
      if n == c and not corner[c] then corner[c] = w end
    end
  end
  -- twin wheels (RL1, RL2, ...): the first one found on each corner
  for _, c in ipairs(CORNERS) do
    if not corner[c] then
      for _, w in pairs(wheels.wheels) do
        local n = w and w.name and string.upper(tostring(w.name))
        if n and n:sub(1, 2) == c then corner[c] = w; break end
      end
    end
  end
end

local function allCorners()
  return corner.FL ~= nil and corner.FR ~= nil and corner.RL ~= nil and corner.RR ~= nil
end

local function loadSurfaces()
  matClass, matWet = {}, {}
  if not (particles and particles.getMaterialsParticlesTable) then return end
  local mats = particles.getMaterialsParticlesTable()
  for id, m in pairs(mats or {}) do
    local n = m and m.name and string.upper(tostring(m.name))
    if n then
      local c = SURFACE[n]
      if c == nil then
        c = 0
        if not unknownSurface[n] then
          unknownSurface[n] = true
          say("I", "ground '" .. n .. "' (" .. tostring(id) .. ") is read as tarmac")
        end
      end
      matClass[id] = c
      matWet[id] = WET[n] and 1 or 0
    end
  end
end

local function clean(s)
  return (tostring(s or ""):gsub("[,=\r\n]", " "):sub(1, 47))
end

-- The car as built: its parts and tuning values, sorted, hashed (8 hex
-- digits), the same every time for the same build; nil when the game
-- gives neither.
local function buildHash(cfg)
  local items = {}
  for k, x in pairs(type(cfg.parts) == "table" and cfg.parts or {}) do items[#items + 1] = "p" .. tostring(k) .. "=" .. tostring(x) end
  for k, x in pairs(type(cfg.vars) == "table" and cfg.vars or {}) do items[#items + 1] = "v" .. tostring(k) .. "=" .. tostring(x) end
  if #items == 0 then return nil end
  table.sort(items)
  local h = 0
  for _, s in ipairs(items) do
    for i = 1, #s do h = (h * 31 + s:byte(i)) % 4294967296 end
    h = (h * 31 + 59) % 4294967296
  end
  return string.format("%08x", h)
end

-- Whether the car differs from the config it was loaded from (a part or a
-- tuning value changed since, in the parts or tuning menu): every slot and
-- value the saved file names, against the car. The parts are compared only
-- when the car lists its own (BeamNG 0.40 keeps them elsewhere: an empty
-- list against the file read every config as changed). nil when the file
-- cannot be read, or there is nothing to compare.
local function changedFromSaved(cfg, path)
  if not (jsonReadFile and path ~= "") then return nil end
  local ok, saved = pcall(jsonReadFile, path)
  if not ok or type(saved) ~= "table" then return nil end
  local parts = type(cfg.parts) == "table" and cfg.parts or {}
  local vars  = type(cfg.vars) == "table" and cfg.vars or {}
  if next(parts) == nil and next(vars) == nil then return nil end
  if next(parts) ~= nil then
    for slot, part in pairs(type(saved.parts) == "table" and saved.parts or {}) do
      if tostring(parts[slot] or "") ~= tostring(part or "") then return true end
    end
  end
  -- the tuning values both name (one only the file names is not a change)
  for k, x in pairs(type(saved.vars) == "table" and saved.vars or {}) do
    if vars[k] ~= nil then
      local a, b = tonumber(vars[k]), tonumber(x)
      if a and b then
        if math.abs(a - b) > 1e-6 * math.max(1, math.abs(b)) then return true end
      elseif tostring(vars[k]) ~= tostring(x) then return true end
    end
  end
  return false
end

-- The car's name and id. The name is what the game shows (the vehicle and
-- its config: "scintilla race"), what nullCAT binds profiles to. The id is
-- unique and stable (the vehicle and its config: "scintilla/race", as two
-- vehicles can each have a "race" config); a car changed from its saved
-- config, or built with none, adds its build's hash ("scintilla/race+1a2b3c4d"):
-- to nullCAT a different car, whose redline, gear ratios and tyres it
-- learns afresh.
local buildNote = ""
local function findCar()
  local cfg  = (v and type(v.config) == "table") and v.config or {}
  local main = tostring(cfg.mainPartName or "")
  local path = tostring(cfg.partConfigFilename or "")
  local base = path:match("([^/\\]+)%.pc$") or ""
  local id   = main .. ((base ~= "" and base ~= main) and ("/" .. base) or "")
  local changed = nil
  if base ~= "" then
    local ok, c = pcall(changedFromSaved, cfg, path)
    if ok then changed = c end
  end
  local hash = nil
  if changed ~= false then
    local ok, hh = pcall(buildHash, cfg)
    if ok then hash = hh end
  end
  local suffix = (hash and (changed == true or base == "")) and ("+" .. hash) or ""
  buildNote = (base == "" and "built without a saved config")
           or (changed == true and "changed from its saved config")
           or (changed == false and "as saved")
           or "its saved config could not be read"
  carId   = clean(id):sub(1, 47 - #suffix) .. suffix   -- a long name gives way, the build's hash stays
  carName = clean(main .. ((base ~= "" and base ~= main) and (" " .. base) or ""))
  engine  = powertrain and powertrain.getDevice and powertrain.getDevice("mainEngine") or nil
end

-- What the car is (protocol 1.11), sent with its name, for nullCAT's Engine
-- and Limiter tiles and its type of car where its car table has no entry:
-- the cylinders the engine's sound is built on, the layout from the
-- sound's name (I4, V8, boxer, rotary), the displacement where that name
-- gives it, else a rough figure from the torque (turbo or not), the turbo,
-- the limiter's cut rate, and the type of car from the config's name
-- (rally gravel or tarmac, race, track, else road). Mod cars carry the same
-- sound and engine data, so they are described the same way.
local carFacts = ""
local function lower(x) return string.lower(tostring(x or "")) end
local function describeCar()
  carFacts = ""
  local data = v and v.data
  local f = {}
  local me = (type(data) == "table" and type(data.mainEngine) == "table") and data.mainEngine or nil
  local electric = false
  if engine == nil and powertrain and powertrain.getDevices then
    local ok, devs = pcall(powertrain.getDevices)
    if ok and type(devs) == "table" then
      for _, d in pairs(devs) do
        if type(d) == "table" and lower(d.type):find("electricmotor") then electric = true end
      end
    end
  end
  if electric then
    f[#f + 1] = "engLayout=5"
  elseif me then
    local sc = (type(me.soundConfig) == "string" and type(data[me.soundConfig]) == "table") and data[me.soundConfig] or nil
    local names = lower(sc and sc.sampleName) .. " " .. lower(me.starterSample) .. " " .. lower(me.shutOffSampleEngine)
    -- a word of the sound's name: at its start, or after a separator
    local function word(p) return names:find("^" .. p) ~= nil or names:find("[%s_>%-]" .. p) ~= nil end
    local layout = nil
    if word("rotary") or word("wankel") then layout = 3
    elseif word("2stroke") or word("two_?stroke") then layout = 4
    elseif word("v%d") then layout = 1
    elseif word("b%d") or word("box") or word("flat%d") then layout = 2
    elseif word("i%d") or word("inline") then layout = 0
    end
    local cyl = sc and tonumber(sc.fundamentalFrequencyCylinderCount)
    if not cyl then cyl = tonumber(names:match("^[vib](%d+)") or names:match("[%s_>%-][vib](%d+)") or names:match("box(%d+)")) end
    if cyl and cyl >= 1 and cyl <= 16 then f[#f + 1] = string.format("engCylinders=%d", cyl) end
    if layout then f[#f + 1] = string.format("engLayout=%d", layout) end
    local turbo = me.turbocharger ~= nil and me.turbocharger ~= ""
    local cc = tonumber(names:match("(%d+)cc"))
    local litres = (cc and cc >= 50 and cc <= 10000) and cc / 1000 or nil
    if not litres then
      local tq = engine and tonumber(engine.maxTorque)
      if tq and tq > 0 then
        local perLitre = (lower(me.requiredEnergyType) == "diesel" and 170) or (turbo and 160)
                      or ((me.supercharger ~= nil and me.supercharger ~= "") and 135) or 100
        litres = math.max(0.6, math.min(8.5, tq / perLitre))
      end
    end
    if litres then f[#f + 1] = string.format("engLitres=%.1f", litres) end
    f[#f + 1] = "engTurbo=" .. (turbo and "1" or "0")
    local cut = tonumber(me.revLimiterCutTime)
    if lower(me.revLimiterType) == "timebased" and cut and cut > 0 then
      f[#f + 1] = string.format("engLimitHz=%.0f", math.max(4, math.min(30, 1 / (1.5 * cut))))
    end
  end
  -- the type of car from the words of the config's and the vehicle's names
  local cfg = (v and type(v.config) == "table") and v.config or {}
  local words = {}
  for w in (lower(cfg.partConfigFilename):match("([^/\\]+)%.pc$") or ""):gmatch("[%a%d]+") do words[w] = true end
  for w in lower(cfg.mainPartName):gmatch("[%a%d]+") do words[w] = true end
  local carType = 0
  if words.rally then carType = (words.tarmac or words.asphalt) and 5 or 4
  elseif words.race or words.racing or words.gt3 or words.gt2 or words.gt4 or words.gte or words.lmp or words.endurance then carType = 2
  elseif words.track or words.trackday or words.cup or words.hillclimb or words.timeattack then carType = 1
  end
  f[#f + 1] = "carType=" .. carType
  carFacts = table.concat(f, ",")
end

-- The body's frame: the world to the car (y forward, z up, x right), for
-- node positions (offsets from the car in the world's directions).
local function bodyFrame()
  return quatFromDir(obj:getDirectionVector(), obj:getDirectionVectorUp()):inversed()
end

local function measureCar()
  wheelbase, trackWidth = nil, nil
  if not allCorners() then return end
  local q = bodyFrame()
  local p = {}
  for _, c in ipairs(CORNERS) do p[c] = q * vec3(obj:getNodePosition(corner[c].node1)) end
  wheelbase  = 0.5 * (p.FL.y + p.FR.y) - 0.5 * (p.RL.y + p.RR.y)
  trackWidth = 0.5 * (math.abs(p.FL.x - p.FR.x) + math.abs(p.RL.x - p.RR.x))
end

-- ---- what BeamNG says about the engine, once per car, to the log
-- (beamng.log in the user folder, lines "nullCAT: ..."): how the car is
-- built against its saved config and where the game keeps the build, the
-- vehicle's information, the engine's sound, turbo and supercharger
-- sections, its jbeam section, the engine device, the powertrain's devices,
-- and the names of every electrics value. Each part has its own cap, so a
-- long one never crowds out the next (about 600 lines in all).

local dumpedFor = nil

local function isScalar(x)
  local t = type(x)
  return t == "number" or t == "string" or t == "boolean"
end

local function dumpSection(label, t, depth, budget)
  if type(t) ~= "table" then return end
  local items = {}
  for k, val in pairs(t) do items[#items + 1] = { tostring(k), val } end
  table.sort(items, function(a, b) return a[1] < b[1] end)
  for _, it in ipairs(items) do
    if budget.n <= 0 then say("I", label .. ": (more not shown)"); return end
    local k, val = it[1], it[2]
    if isScalar(val) then
      say("I", label .. "." .. k .. " = " .. tostring(val)); budget.n = budget.n - 1
    elseif type(val) == "table" then
      local n = 0
      for _ in pairs(val) do n = n + 1 end
      if depth > 0 and n <= 40 then
        dumpSection(label .. "." .. k, val, depth - 1, budget)
      else
        say("I", label .. "." .. k .. " = table of " .. n); budget.n = budget.n - 1
      end
    end
  end
end

local function dumpEngine()
  dumpedFor = carId
  say("I", "---- engine of " .. carName .. " (" .. carId .. ") ----")
  local cfg = (v and type(v.config) == "table") and v.config or {}
  local np, nv = 0, 0
  for _ in pairs(type(cfg.parts) == "table" and cfg.parts or {}) do np = np + 1 end
  for _ in pairs(type(cfg.vars) == "table" and cfg.vars or {}) do nv = nv + 1 end
  say("I", "build: " .. buildNote .. ", " .. np .. " part slots, " .. nv .. " tuning values, config " .. tostring(cfg.partConfigFilename))
  -- where the game keeps the build: the names in the config and the jbeam data
  local function keyList(label, t)
    if type(t) ~= "table" then return end
    local ks = {}
    for k, x in pairs(t) do
      local n = 0
      if type(x) == "table" then for _ in pairs(x) do n = n + 1 end end
      ks[#ks + 1] = tostring(k) .. ((type(x) == "table") and ("(" .. n .. ")") or "")
    end
    table.sort(ks)
    for i = 1, #ks, 20 do say("I", label .. ": " .. table.concat(ks, " ", i, math.min(i + 19, #ks))) end
  end
  keyList("config", cfg)
  local data = v and v.data
  keyList("jbeam", data)
  say("I", "described as: " .. carFacts)
  -- the config's own info file (its body style, its type and so on), for the type of car
  local model = tostring(cfg.model or cfg.mainPartName or "")
  local base = tostring(cfg.partConfigFilename or ""):match("([^/\\]+)%.pc$")
  if jsonReadFile and model ~= "" and base then
    local ok, info = pcall(jsonReadFile, "/vehicles/" .. model .. "/info_" .. base .. ".json")
    if ok and type(info) == "table" then dumpSection("info", info, 0, { n = 40 }) else say("I", "no info file for the config") end
  end
  if type(data) == "table" then
    local me = data.mainEngine
    dumpSection("jbeam.information", data.information, 1, { n = 60 })
    if type(me) == "table" then
      for _, k in ipairs({ "soundConfig", "soundConfigExhaust", "turbocharger", "supercharger", "nitrousOxideInjection" }) do
        local ref = me[k]
        if type(ref) == "string" and type(data[ref]) == "table" then dumpSection("jbeam." .. ref, data[ref], 1, { n = 80 }) end
      end
    end
    for _, k in ipairs({ "soundConfig", "turbocharger", "supercharger" }) do
      if type(data[k]) == "table" then dumpSection("jbeam." .. k, data[k], 1, { n = 60 }) end
    end
    dumpSection("jbeam.mainEngine", me, 0, { n = 120 })
  end
  if engine then dumpSection("device", engine, 0, { n = 120 }) else say("I", "no mainEngine device") end
  if powertrain and powertrain.getDevices then
    local ok, devs = pcall(powertrain.getDevices)
    if ok and type(devs) == "table" then
      for name, d in pairs(devs) do
        say("I", "powertrain " .. tostring(name) .. ": " .. tostring(type(d) == "table" and d.type or type(d)))
      end
    end
  end
  local e = electrics and electrics.values
  if type(e) == "table" then
    local ks = {}
    for k in pairs(e) do ks[#ks + 1] = tostring(k) end
    table.sort(ks)
    for i = 1, #ks, 25 do say("I", "electrics: " .. table.concat(ks, " ", i, math.min(i + 24, #ks))) end
  end
  say("I", "---- end of engine ----")
end

-- ---- the line

local parts = {}
local slowSec, idSec = 1e9, 1e9   -- the first line carries both
local buildFailed = false

local function add(key, x, fmt)
  x = tonumber(x)
  if x == nil or x ~= x or x == math.huge or x == -math.huge then return end
  parts[#parts + 1] = key .. "=" .. string.format(fmt, x)
end

local function flag(x)
  if x == true then return 1 end
  if x == nil or x == false then return 0 end
  return ((tonumber(x) or 0) ~= 0) and 1 or 0
end

local function build(dt)
  for i = #parts, 1, -1 do parts[i] = nil end
  parts[1] = "NULLCATY"
  slowSec, idSec = slowSec + dt, idSec + dt
  local e = (electrics and electrics.values) or {}

  -- the car's name, about once a second
  if idSec >= ID_EVERY_SEC then
    idSec = 0
    parts[#parts + 1] = "game=" .. GAME
    parts[#parts + 1] = "car=" .. carName
    parts[#parts + 1] = "carId=" .. carId
    parts[#parts + 1] = "direct=only"
    if carFacts ~= "" then parts[#parts + 1] = carFacts end
    -- once per car, after its first second (the electrics are filled by then)
    if DUMP_ENGINE and dumpedFor ~= carId and slowSec < 1e8 then
      local ok, err = pcall(dumpEngine)
      if not ok then dumpedFor = carId; say("E", "engine dump stopped: " .. tostring(err)) end
    end
  end

  -- speed over the ground: the wheels' own speed would hide a lock or a spin
  local vel = obj:getVelocity()
  add("speedKmh", math.sqrt(vel.x * vel.x + vel.y * vel.y) * 3.6, "%.2f")
  add("rpm", e.rpm, "%.0f")
  add("gear", e.gearIndex, "%.0f")
  add("throttlePct", (tonumber(e.throttle) or 0) * 100, "%.1f")
  add("brakePct", (tonumber(e.brake) or 0) * 100, "%.1f")
  add("clutchPct", (tonumber(e.clutch) or 0) * 100, "%.1f")
  add("absActive", flag(e.absActive), "%.0f")
  -- traction control alone: stability control brakes a wheel off the
  -- throttle, which is not TC cutting the power (it read as TC on lifts)
  add("tcActive", flag(e.tcsActive), "%.0f")
  -- the limiter as the engine has it: each cut drops the rpm well below
  -- the limit (200 rpm on a rally car), so the rpm alone rarely read it;
  -- held over the short recoveries between cuts. The rpm against the
  -- limit only where the game says nothing.
  local maxRpm = engine and (tonumber(engine.revLimiterRPM) or tonumber(engine.maxRPM)) or nil
  if maxRpm and maxRpm <= 0 then maxRpm = nil end
  local onLimiter = nil
  if engine and engine.revLimiterActive ~= nil then
    local since = tonumber(engine.revLimiterWasActiveTimer)
    onLimiter = engine.revLimiterActive == true or (since ~= nil and since < LIMITER_HOLD_SEC)
  elseif maxRpm then
    onLimiter = (tonumber(e.rpm) or 0) >= 0.99 * maxRpm
  end
  if onLimiter ~= nil then add("limiter", onLimiter and 1 or 0, "%.0f") end

  -- the body: its acceleration, the rates it turns at, its angles. The
  -- sensor already reads in the car's own frame (no turning it): along
  -- above zero braking, across with the sign the other games give it
  -- (below zero in a right-hand corner), up above zero, gravity out.
  -- Checked against the speed, the yaw rate, the tyre loads and the road
  -- on recorded drives; the dash's smoothed g runs 0.1-0.3 s late.
  local q = bodyFrame()
  local p = protocols or {}
  add("accSurge", obj:getSensorY(), "%.3f")
  add("accSway", obj:getSensorX(), "%.3f")
  add("accHeave", -obj:getSensorZ(), "%.3f")
  add("yawRate", obj:getYawAngularVelocity() * RAD2DEG, "%.2f")
  add("pitchRate", obj:getPitchAngularVelocity() * RAD2DEG, "%.2f")
  add("rollRate", obj:getRollAngularVelocity() * RAD2DEG, "%.2f")
  local pitch, roll = p.pitchPosSmoothed or p.pitchPos, p.rollPosSmoothed or p.rollPos
  if pitch ~= nil then add("pitchDeg", pitch * RAD2DEG, "%.2f") end
  if roll ~= nil then add("rollDeg", roll * RAD2DEG, "%.2f") end

  -- each tyre
  local slow = slowSec >= SLOW_EVERY_SEC
  if allCorners() then
    local pos = obj:getPosition()
    for _, c in ipairs(CORNERS) do
      local w = corner[c]
      local id = tonumber(w.contactMaterialID1) or -1
      add("wheelSpeed" .. c, w.angularVelocity, "%.3f")
      add("load" .. c, w.downForceRaw, "%.0f")
      -- how fast the contact patch slides over the road, m/s (nullCAT
      -- works the slip angle out of it)
      add("slipSpeed" .. c, w.lastSlip, "%.3f")
      add("onGround" .. c, (id >= 0) and 1 or 0, "%.0f")
      if id >= 0 then
        add("surface" .. c, matClass[id] or 0, "%.0f")
        if slow then add("wet" .. c, matWet[id] or 0, "%.0f") end
      end
      local np = obj:getNodePosition(w.node1)
      add("suspTravel" .. c, (q * vec3(np)).z * 1000, "%.2f")
      if SEND_HEIGHTS then
        local h = obj:getSurfaceHeightBelow(vec3(pos.x + np.x, pos.y + np.y, pos.z + np.z + 2))
        if h ~= nil and h > -1e10 then add("roadHeight" .. c, h * 1000, "%.1f") end
      end
      if slow then add("deflated" .. c, flag(w.isTireDeflated), "%.0f") end
      if DEBUG_KEYS then
        add("dSlipE" .. c, w.slipEnergy, "%.2f")
        add("dWs" .. c, w.wheelSpeed, "%.3f")
      end
    end
  end

  if slow then
    slowSec = 0
    if maxRpm and maxRpm > 0 then add("maxRpm", maxRpm, "%.0f") end
    if wheelbase then add("wheelbase", wheelbase, "%.3f") end
    if trackWidth then add("trackWidth", trackWidth, "%.3f") end
    if DEBUG_KEYS then
      add("dHasAbs", flag(e.hasABS), "%.0f")
      add("dTurbo", e.turboBoost, "%.3f")
      add("dBoost", e.boost, "%.3f")
      add("dClutchRatio", e.clutchRatio, "%.3f")
      add("dWheelspeed", e.wheelspeed, "%.3f")
      add("dAirspeed", e.airspeed, "%.3f")
      add("dRate", rate(), "%.0f")
    end
  end

  if DEBUG_KEYS then
    -- the raw sensor with its own signs, and the dash's smoothed g
    -- (sensors), to check the signs above again
    add("dSx", -obj:getSensorX(), "%.3f"); add("dSy", -obj:getSensorY(), "%.3f"); add("dSz", -obj:getSensorZ(), "%.3f")
    local s = sensors or {}
    add("dGx", s.gx2, "%.3f"); add("dGy", s.gy2, "%.3f"); add("dGz", s.gz2, "%.3f")
    add("dDt", dt * 1000, "%.2f")
  end
  return table.concat(parts, ",")
end

-- ---- BeamNG's protocol interface

-- What the car is, again (a load or a reset); a step that fails is named
-- in the log and the line goes on without what it would have found.
local function learnCar(surfaces)
  local steps = surfaces and { findWheels, loadSurfaces, findCar, describeCar, measureCar } or { findWheels, findCar, describeCar, measureCar }
  for _, step in ipairs(steps) do
    local ok, err = pcall(step)
    if not ok then say("E", "could not read the car: " .. tostring(err)) end
  end
  slowSec, idSec = 1e9, 1e9
end

local function init()
  learnCar(true)
  say("I", "sending to " .. tostring(setting("address", ADDRESS)) .. ":" .. tostring(setting("port", PORT))
        .. " at " .. rate() .. " a second" .. ((rate() > 100) and " (physics step)" or " (per frame)"))
end

local function reset()
  learnCar(false)
end

local function getAddress()       return setting("address", ADDRESS) end
local function getPort()          return tonumber(setting("port", PORT)) or PORT end
local function getMaxUpdateRate() return rate() end
local function isPhysicsStepUsed() return rate() > 100 end

local function getStructDefinition()
  return "char text[" .. LINE_LEN .. "];"
end

-- Whether a player sits in this car. Another car (traffic, a parked one)
-- says once that it has no direct source (the car you get out of lets
-- nullCAT go), then sends only padding, which nullCAT drops unread: it
-- never stands in for the car you drive. A game that does not say counts
-- as seated.
local wasSeated, sayLeft = true, false
local function seated()
  local p = playerInfo
  local s = not (type(p) == "table" and p.anyPlayerSeated == false)
  if s ~= wasSeated then
    wasSeated = s
    if not s then
      sayLeft = true
      say("I", "no player in " .. carName .. ": its telemetry stops")
    end
    idSec = 1e9   -- back in the car: its name goes with the first line
  end
  return s
end

local function fillStruct(o, dtSim)
  local dt = tonumber(dtSim) or 0
  if dt <= 0 then dt = 1 / rate() end   -- no step given: the nominal one
  local line
  if seated() then
    local ok, l = pcall(build, dt)
    line = l
    if not ok then
      if not buildFailed then
        buildFailed = true
        say("E", "could not build its line: " .. tostring(l))
      end
      line = "NULLCATY,game=" .. GAME
    end
  elseif sayLeft then
    sayLeft = false
    line = "NULLCATY,direct=none"
  else
    line = ""
  end
  ffi.fill(o.text, LINE_LEN, 32)
  ffi.copy(o.text, line, math.min(#line, LINE_LEN))
end

M.init = init
M.reset = reset
M.getAddress = getAddress
M.getPort = getPort
M.getMaxUpdateRate = getMaxUpdateRate
M.isPhysicsStepUsed = isPhysicsStepUsed
M.getStructDefinition = getStructDefinition
M.fillStruct = fillStruct

return M
