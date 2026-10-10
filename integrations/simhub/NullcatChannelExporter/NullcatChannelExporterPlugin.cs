// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
//
// nullCAT Channel Exporter - a deliberately dumb SimHub plugin.
//
// Sends up to three UDP lines per data tick (protocol 1.4, Docs/PROTOCOL.md):
//
//   NULLCATX,<rpm>,<speedKmh>,<gear>,<clutchPct>,<throttlePct>,
//            <brakePct>,<absActive>,<skid>,<lockup>,<roadNoise>,
//            <limiter>,<tcActive>,<curbs>,<maxRpm>      (for older receivers)
//
//   NULLCATY,rpm=..,speedKmh=..,...,boost=..,pitLimiter=..,wheelSpeedFL=..,...
//            (everything by name: the classic channels again, boost and the
//            pit limiter, the magnitudes when bound, the per-wheel groups
//            in force, each only when all four wheels read)
//
//   NULLCATY,game=..,car=..,carId=..               (once a second)
//
// That is the whole job. No shaping, no state: nullCAT owns the tyre
// model and maps channels onto effects. The only game knowledge here is
// WHICH raw fields each sim exposes per wheel (the built-in presets), so
// a known sim needs no setup at all; the settings file overrides a group
// when it binds all four wheels. A group that is not in force is simply
// absent (never a row of zeros a receiver could mistake for "no slip").
//
// Configuration: NullcatChannelExporter.json next to this DLL,
//   { "host": "192.168.1.50", "port": 4444,
//     "skidProp": "", "lockupProp": "", "roadProp": "", "curbsProp": "",
//     "slipAngleFLProp": "", ... "slipAngleRRProp": "", "slipAngleScale": 1,
//     "slipRatioFLProp": "", ...,                        "slipRatioScale": 1,
//     "wheelSpeedFLProp": "", ...,                       "wheelSpeedScale": 1,
//     "loadFLProp": "", ...,                             "loadScale": 1,
//     "suspVelFLProp": "", ...,                          "suspVelScale": 1 }
// Defaults to 127.0.0.1:4444 when the file is absent or unreadable.
// Every *Prop key is an OPTIONAL full SimHub property name (any property,
// including NCalc-computed ones and the game's raw data). The magnitude
// props (skid, lockup, road, curbs) are expected to yield 0..100. The
// per-wheel props are RAW physics: slip angle in degrees, slip ratio
// signed, wheel speed in any unit, load in any unit, suspension velocity
// in mm/s; each group has a scale for unit conversion (57.2958 turns
// radians into degrees, 1000 turns m/s into mm/s). A group is sent only
// when all four wheels are bound.

using System;
using System.Globalization;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Reflection;
using System.Text;
using GameReaderCommon;
using SimHub.Plugins;

namespace NullcatChannelExporter
{
    [PluginDescription("Sends raw telemetry to nullCAT over UDP: rpm, speed, gear, pedals, ABS/TC/limiter flags, and optional per-wheel slip, load and suspension channels for the haptic layer")]
    [PluginAuthor("nullCAT")]
    [PluginName("nullCAT Channel Exporter")]
    public class NullcatChannelExporterPlugin : IPlugin, IDataPlugin
    {
        public PluginManager PluginManager { get; set; }

        private UdpClient _udp;
        private IPEndPoint _target;
        private string _skidProp, _lockupProp, _roadProp, _curbsProp;

        // Per-wheel groups, wheel order FL FR RL RR (the protocol's).
        private static readonly string[] Wheels = { "FL", "FR", "RL", "RR" };
        private static readonly string[] Groups = { "slipAngle", "slipRatio", "wheelSpeed", "load", "suspVel", "wheelSlip", "suspTravel" };
        private readonly string[][] _fileProps  = new string[Groups.Length][];   // from the settings file (override)
        private readonly double[]   _fileScale  = { 1, 1, 1, 1, 1, 1, 1 };
        private readonly string[][] _groupProps = new string[Groups.Length][];   // what is in force (file, else preset)
        private readonly double[]   _groupScale = { 1, 1, 1, 1, 1, 1, 1 };

        // Built-in bindings per game, so nothing needs typing for the sims
        // we know: the raw per-wheel fields SimHub exposes for each (read
        // from its game-reader assemblies; 01 = FL, 02 = FR, 03 = RL,
        // 04 = RR in every one of them) with the unit scale the wire wants.
        // The settings file overrides a group only when it binds all four
        // wheels. Keyed by SimHub's GameName, compared without case or
        // punctuation.
        private struct PresetGroup { public string Group, Prefix; public double Scale; public PresetGroup(string g, string p, double s) { Group = g; Prefix = p; Scale = s; } }
        // Assetto Corsa's combined slip reads about 1 to 2 as the tyre starts to
        // slide and 10 and up in a spin (SimHub's own slip LEDs map 1..10):
        // 6 = let go = 100 on the wire, a starting point the rig's peak % trims.
        private const double AcSlipScale = 100.0 / 6.0;
        private static readonly PresetGroup[] PresetAC = {
            // Assetto Corsa: angular speed (rad/s) for lock/spin, load, the
            // combined slip for lateral, travel (m -> mm).
            new PresetGroup("wheelSpeed", "DataCorePlugin.GameRawData.Physics.WheelAngularSpeed", 1),
            new PresetGroup("load",       "DataCorePlugin.GameRawData.Physics.WheelLoad",         1),
            new PresetGroup("wheelSlip",  "DataCorePlugin.GameRawData.Physics.WheelSlip",         AcSlipScale),
            new PresetGroup("suspTravel", "DataCorePlugin.GameRawData.Physics.SuspensionTravel",  1000),
        };
        private static readonly PresetGroup[] PresetACC = {
            // Competizione, EVO, Rally: a real slip ratio per wheel, the rest as AC.
            new PresetGroup("slipRatio",  "DataCorePlugin.GameRawData.Physics.slipRatio",         1),
            new PresetGroup("wheelSpeed", "DataCorePlugin.GameRawData.Physics.WheelAngularSpeed", 1),
            new PresetGroup("load",       "DataCorePlugin.GameRawData.Physics.WheelLoad",         1),
            new PresetGroup("wheelSlip",  "DataCorePlugin.GameRawData.Physics.WheelSlip",         AcSlipScale),
            new PresetGroup("suspTravel", "DataCorePlugin.GameRawData.Physics.SuspensionTravel",  1000),
        };
        private static readonly PresetGroup[] PresetAMS2 = {
            // Automobilista 2 (shared memory): tyre rev/s for lock/spin,
            // suspension velocity (m/s -> mm/s), slip speed (m/s; 5 m/s -> 100).
            new PresetGroup("wheelSpeed", "DataCorePlugin.GameRawData.mTyreRPS",            1),
            new PresetGroup("suspVel",    "DataCorePlugin.GameRawData.mSuspensionVelocity", 1000),
            new PresetGroup("wheelSlip",  "DataCorePlugin.GameRawData.mTyreSlipSpeed",      20),
        };
        private static PresetGroup[] PresetFor(string gameName)
        {
            var g = Norm(gameName);
            if (g == "assettocorsa") return PresetAC;
            if (g == "assettocorsacompetizione" || g == "assettocorsaevo" || g == "assettocorsarally") return PresetACC;
            if (g == "automobilista2") return PresetAMS2;
            return null;
        }
        private static string Norm(string s)
        {
            if (string.IsNullOrEmpty(s)) return "";
            var sb = new StringBuilder(s.Length);
            foreach (var c in s) if (char.IsLetterOrDigit(c)) sb.Append(char.ToLowerInvariant(c));
            return sb.ToString();
        }
        private string _presetGame;   // the game the groups in force were resolved for

        private DateTime _lastIdentity = DateTime.MinValue;

        public void Init(PluginManager pluginManager)
        {
            _pm = pluginManager;
            var host = "127.0.0.1";
            var port = 4444;
            var settingsNote = "no NullcatChannelExporter.json beside the DLL, defaults";
            for (var g = 0; g < Groups.Length; g++) { _fileProps[g] = new string[4]; _groupProps[g] = new string[4]; }
            try
            {
                var dir  = Path.GetDirectoryName(typeof(NullcatChannelExporterPlugin).Assembly.Location);
                var path = Path.Combine(dir ?? ".", "NullcatChannelExporter.json");
                if (File.Exists(path))
                {
                    settingsNote = "settings from " + path;
                    // Tiny hand parser: known keys only, no JSON library needed.
                    var text = File.ReadAllText(path);
                    var h = ExtractString(text, "host");
                    if (!string.IsNullOrWhiteSpace(h)) host = h.Trim();
                    var p = ExtractNumber(text, "port");
                    if (p > 0 && p < 65536) port = (int)p;
                    _skidProp   = ExtractString(text, "skidProp");
                    _lockupProp = ExtractString(text, "lockupProp");
                    _roadProp   = ExtractString(text, "roadProp");
                    _curbsProp  = ExtractString(text, "curbsProp");
                    for (var g = 0; g < Groups.Length; g++)
                    {
                        for (var w = 0; w < 4; w++)
                            _fileProps[g][w] = ExtractString(text, Groups[g] + Wheels[w] + "Prop");
                        var sc = ExtractNumber(text, Groups[g] + "Scale");
                        if (sc > 0) _fileScale[g] = sc;
                    }
                }
            }
            catch (Exception ex) { settingsNote = "settings file unreadable (" + ex.Message + "), defaults"; }
            ApplyBindings(null);

            try { _target = new IPEndPoint(IPAddress.Parse(host), port); }
            catch (Exception ex)
            {
                settingsNote += "; host '" + host + "' is not an IP address (" + ex.Message + "), sending to 127.0.0.1";
                host = "127.0.0.1";
                _target = new IPEndPoint(IPAddress.Loopback, port);
            }
            _udp = new UdpClient();

            // Say where the stream goes, in SimHub's log and as a property
            // anyone can read in the property list: when nothing arrives at
            // the rig, this is the first thing to look at.
            _targetText = host + ":" + port;
            try { SimHub.Logging.Current.Info("nullCAT Channel Exporter: sending to " + _targetText + " (" + settingsNote + ")"); } catch { }
            try { pluginManager.AddProperty("Target", GetType(), _targetText); } catch { }
            try { pluginManager.AddProperty("Game", GetType(), ""); } catch { }
        }
        private string _targetText = "";

        // The groups in force for a game: the file's where it binds all
        // four wheels, else the built-in preset's, else nothing.
        private void ApplyBindings(string gameName)
        {
            var preset = PresetFor(gameName);
            for (var g = 0; g < Groups.Length; g++)
            {
                var f = _fileProps[g];
                var fileBound = !string.IsNullOrWhiteSpace(f[0]) && !string.IsNullOrWhiteSpace(f[1])
                             && !string.IsNullOrWhiteSpace(f[2]) && !string.IsNullOrWhiteSpace(f[3]);
                if (fileBound)
                {
                    for (var w = 0; w < 4; w++) _groupProps[g][w] = f[w];
                    _groupScale[g] = _fileScale[g];
                    continue;
                }
                for (var w = 0; w < 4; w++) _groupProps[g][w] = null;
                _groupScale[g] = 1;
                if (preset == null) continue;
                foreach (var pg in preset)
                {
                    if (pg.Group != Groups[g]) continue;
                    for (var w = 0; w < 4; w++) _groupProps[g][w] = pg.Prefix + "0" + (w + 1);
                    _groupScale[g] = pg.Scale;
                }
            }
            _presetGame = gameName ?? "";
            if (_presetGame.Length > 0)
            {
                var groups = new StringBuilder();
                for (var g = 0; g < Groups.Length; g++)
                    if (!string.IsNullOrWhiteSpace(_groupProps[g][0])) groups.Append(groups.Length > 0 ? ", " : "").Append(Groups[g]);
                var note = "nullCAT Channel Exporter: game '" + _presetGame + "', per-wheel groups: "
                         + (groups.Length > 0 ? groups.ToString() : "none") + (preset != null ? " (built-in preset)" : " (no preset for this game)");
                try { SimHub.Logging.Current.Info(note); } catch { }
                try { _pm?.SetPropertyValue("Game", GetType(), _presetGame + ": " + (groups.Length > 0 ? groups.ToString() : "no per-wheel groups")); } catch { }
            }
        }
        private PluginManager _pm;

        public void DataUpdate(PluginManager pluginManager, ref GameData data)
        {
            if (_udp == null || !data.GameRunning || data.NewData == null) return;
            // A paused game keeps reporting its last frame; sending it would
            // hold an rpm vibration on through the pause menu. Stay silent
            // and nullCAT's staleness fail-safe releases every effect.
            try { if (data.GamePaused) return; } catch { }
            var d = data.NewData;
            // A different game: its built-in bindings (the file's override
            // groups stay in force whatever the game).
            if ((data.GameName ?? "") != _presetGame) ApplyBindings(data.GameName);

            // Gear arrives as a string ("N", "R", "1".."8"); the wire wants a
            // number: N -> 0, R -> -1, digits as-is, anything odd -> 0.
            double gear = 0;
            var g0 = d.Gear;
            if (!string.IsNullOrEmpty(g0))
            {
                if (g0 == "R" || g0 == "r") gear = -1;
                else double.TryParse(g0, NumberStyles.Integer, CultureInfo.InvariantCulture, out gear);
            }

            // ABS / TC flags: StatusDataBase carries them where the game
            // reports them; anything nonzero on the wire means "active".
            double abs = 0, tc = 0;
            try { abs = Convert.ToDouble(d.ABSActive, CultureInfo.InvariantCulture) > 0 ? 1 : 0; }
            catch { abs = 0; }
            try { tc  = Convert.ToDouble(d.TCActive,  CultureInfo.InvariantCulture) > 0 ? 1 : 0; }
            catch { tc = 0; }

            // Rev limiter: bouncing off the top of the tach. Computed here
            // (rpm within 1.5% of the car's max rpm) because few games
            // expose a limiter flag directly; MaxRpm 0/unknown = never on.
            // maxRpm itself also goes on the wire (slot 13); nullCAT learns
            // the redline itself when it is 0.
            double limiter = 0, maxRpm = 0;
            try
            {
                maxRpm = Convert.ToDouble(d.MaxRpm, CultureInfo.InvariantCulture);
                if (double.IsNaN(maxRpm) || double.IsInfinity(maxRpm) || maxRpm < 0) maxRpm = 0;
                if (maxRpm > 0 && d.Rpms >= maxRpm * 0.985) limiter = 1;
            }
            catch { limiter = 0; maxRpm = 0; }

            // Turbo boost in bar (SimHub reports relative pressure where the
            // game has it; 0 for a naturally aspirated car or an unknown).
            double boost = 0;
            try
            {
                boost = Convert.ToDouble(d.Turbo, CultureInfo.InvariantCulture);
                if (double.IsNaN(boost) || double.IsInfinity(boost)) boost = 0;
            }
            catch { boost = 0; }

            var skid   = ReadProp(pluginManager, _skidProp);
            var lockup = ReadProp(pluginManager, _lockupProp);
            var road   = ReadProp(pluginManager, _roadProp);
            var curbs  = ReadProp(pluginManager, _curbsProp);
            var line = string.Format(CultureInfo.InvariantCulture,
                "NULLCATX,{0:0.#},{1:0.##},{2:0},{3:0.#},{4:0.#},{5:0.#},{6:0},{7:0.#},{8:0.#},{9:0.#},{10:0},{11:0},{12:0.#},{13:0}",
                d.Rpms, d.SpeedKmh, gear, d.Clutch, d.Throttle,
                d.Brake, abs, skid, lockup, road, limiter, tc, curbs, maxRpm);
            Send(line);

            // The named line carries EVERYTHING by name: the classic
            // channels again (so a receiver's slot map never matters for
            // this plugin; a name wins over a slot binding), boost and the
            // pit limiter, the four magnitudes only when bound, then the
            // per-wheel groups in force, each only when all four wheels
            // read as numbers this tick. The numbered line above stays for
            // older receivers.
            double pit = 0;
            try { pit = Convert.ToDouble(d.PitLimiterOn, CultureInfo.InvariantCulture) > 0 ? 1 : 0; } catch { pit = 0; }
            var y = new StringBuilder(256);
            y.Append("NULLCATY,rpm=").Append(d.Rpms.ToString("0.#", CultureInfo.InvariantCulture))
             .Append(",speedKmh=").Append(d.SpeedKmh.ToString("0.##", CultureInfo.InvariantCulture))
             .Append(",gear=").Append(gear.ToString("0", CultureInfo.InvariantCulture))
             .Append(",clutchPct=").Append(d.Clutch.ToString("0.#", CultureInfo.InvariantCulture))
             .Append(",throttlePct=").Append(d.Throttle.ToString("0.#", CultureInfo.InvariantCulture))
             .Append(",brakePct=").Append(d.Brake.ToString("0.#", CultureInfo.InvariantCulture))
             .Append(",absActive=").Append(abs.ToString("0", CultureInfo.InvariantCulture))
             .Append(",limiter=").Append(limiter.ToString("0", CultureInfo.InvariantCulture))
             .Append(",tcActive=").Append(tc.ToString("0", CultureInfo.InvariantCulture))
             .Append(",maxRpm=").Append(maxRpm.ToString("0", CultureInfo.InvariantCulture))
             .Append(",boost=").Append(boost.ToString("0.###", CultureInfo.InvariantCulture))
             .Append(",pitLimiter=").Append(pit.ToString("0", CultureInfo.InvariantCulture));
            if (!string.IsNullOrWhiteSpace(_skidProp))   y.Append(",skid=").Append(skid.ToString("0.#", CultureInfo.InvariantCulture));
            if (!string.IsNullOrWhiteSpace(_lockupProp)) y.Append(",lockup=").Append(lockup.ToString("0.#", CultureInfo.InvariantCulture));
            if (!string.IsNullOrWhiteSpace(_roadProp))   y.Append(",roadNoise=").Append(road.ToString("0.#", CultureInfo.InvariantCulture));
            if (!string.IsNullOrWhiteSpace(_curbsProp))  y.Append(",curbs=").Append(curbs.ToString("0.#", CultureInfo.InvariantCulture));
            for (var g = 0; g < Groups.Length; g++)
            {
                var props = _groupProps[g];
                if (string.IsNullOrWhiteSpace(props[0]) || string.IsNullOrWhiteSpace(props[1]) ||
                    string.IsNullOrWhiteSpace(props[2]) || string.IsNullOrWhiteSpace(props[3])) continue;
                var vals = new double[4];
                var ok = true;
                for (var w = 0; w < 4 && ok; w++) ok = ReadRaw(pluginManager, props[w], out vals[w]);
                if (!ok) continue;
                for (var w = 0; w < 4; w++)
                    y.Append(',').Append(Groups[g]).Append(Wheels[w]).Append('=')
                     .Append((vals[w] * _groupScale[g]).ToString("0.####", CultureInfo.InvariantCulture));
            }
            AppendChassis(y, d);
            var family = Norm(data.GameName);
            if (family == "assettocorsa" || family == "assettocorsacompetizione" || family == "assettocorsaevo" || family == "assettocorsarally")
                AppendAcContact(y, d);
            else if (family == "automobilista2")
                AppendAms2Surface(y, pluginManager);
            Send(y.ToString());

            // Identity once a second: which game and car the channels
            // describe (free text; commas and '=' would break the line).
            // car = the name the sim shows (CarModel), carId = SimHub's
            // stable id for it (the content folder in Assetto Corsa), which
            // nullCAT's car table is keyed by.
            var now = DateTime.UtcNow;
            if ((now - _lastIdentity).TotalSeconds >= 1.0)
            {
                _lastIdentity = now;
                var game  = Clean(data.GameName);
                var car   = Clean(d.CarModel);
                var carId = Clean(d.CarId);
                if (game.Length > 0 || car.Length > 0)
                    Send("NULLCATY,game=" + game + ",car=" + car + ",carId=" + carId);
            }
        }

        public void End(PluginManager pluginManager)
        {
            _udp?.Close();
            _udp = null;
        }

        // ---- protocol 1.5: the body and the ground --------------------------

        // The body's vertical acceleration, pitch and roll, from SimHub's own
        // normalized motion data (the same values its motion output uses, so
        // they mean the same thing in every game). Heave is in G there.
        private static void AppendChassis(StringBuilder y, StatusDataBase d)
        {
            double? heave = null;
            try { heave = d.AccelerationHeave; } catch { heave = null; }
            if (!heave.HasValue || double.IsNaN(heave.Value) || double.IsInfinity(heave.Value)) return;
            double pitch, roll;
            try { pitch = d.OrientationPitch; roll = d.OrientationRoll; } catch { return; }
            if (double.IsNaN(pitch) || double.IsNaN(roll)) return;
            y.Append(",accHeave=").Append((heave.Value * 9.80665).ToString("0.###", CultureInfo.InvariantCulture))
             .Append(",pitchDeg=").Append(pitch.ToString("0.####", CultureInfo.InvariantCulture))
             .Append(",rollDeg=").Append(roll.ToString("0.####", CultureInfo.InvariantCulture));
        }

        // Assetto Corsa family: each tyre's contact patch (world position)
        // and the direction it points. From those: the height of the road
        // under each tyre, the car's wheelbase and track, and a TRUE slip
        // angle per wheel (the angle between where the tyre points and where
        // its patch actually moved since the last tick; plain AC has no slip
        // angle field). Competizione and EVO also give the game's own kerb
        // vibration. Read by reflection, so a SimHub update that renames its
        // reader types disables these channels instead of the plugin.
        private void AppendAcContact(StringBuilder y, StatusDataBase d)
        {
            object raw;
            try { raw = d.GetRawDataObject(); } catch { return; }
            var phys = AcPhysics(raw);
            if (phys == null || _acContactPoint == null) return;
            var cp = _acContactPoint.GetValue(phys) as Array;
            var ch = _acContactHeading != null ? _acContactHeading.GetValue(phys) as Array : null;
            var px = new double[4]; var py = new double[4]; var pz = new double[4];
            for (var w = 0; w < 4; w++)
                if (!AcVec(cp, w, out px[w], out py[w], out pz[w])) return;

            // Road height under each tyre, mm (hills and all: nullCAT filters).
            for (var w = 0; w < 4; w++)
                y.Append(",roadHeight").Append(Wheels[w]).Append('=')
                 .Append((py[w] * 1000.0).ToString("0.##", CultureInfo.InvariantCulture));

            // Geometry from the patches: front pair midpoint to rear pair
            // midpoint, and the mean distance across each axle (horizontal).
            var wb = Math.Sqrt(Sq((px[0] + px[1] - px[2] - px[3]) / 2) + Sq((pz[0] + pz[1] - pz[2] - pz[3]) / 2));
            var tw = (Math.Sqrt(Sq(px[0] - px[1]) + Sq(pz[0] - pz[1])) + Math.Sqrt(Sq(px[2] - px[3]) + Sq(pz[2] - pz[3]))) / 2;
            if (wb > 1.0 && wb < 6.0 && tw > 0.8 && tw < 3.0)
                y.Append(",wheelbase=").Append(wb.ToString("0.###", CultureInfo.InvariantCulture))
                 .Append(",trackWidth=").Append(tw.ToString("0.###", CultureInfo.InvariantCulture));

            // Slip angle per wheel, unless the settings file binds its own.
            var fileSlip = !string.IsNullOrWhiteSpace(_fileProps[0][0]);
            if (!fileSlip && ch != null && _hadContact && d.SpeedKmh > 5.0)
            {
                var ang = new double[4];
                var all = true;
                for (var w = 0; w < 4 && all; w++)
                {
                    double hx, hy, hz;
                    if (!AcVec(ch, w, out hx, out hy, out hz)) { all = false; break; }
                    var dx = px[w] - _prevPx[w];
                    var dz = pz[w] - _prevPz[w];
                    var dist = Math.Sqrt(dx * dx + dz * dz);
                    if (dist < 0.005 || dist > 5.0 || (hx == 0 && hz == 0)) { all = false; break; }
                    var a = Math.Atan2(Math.Abs(hx * dz - hz * dx), hx * dx + hz * dz) * 180.0 / Math.PI;
                    ang[w] = a > 90.0 ? 180.0 - a : a;   // rolling backwards: the same slip, mirrored
                }
                if (all)
                    for (var w = 0; w < 4; w++)
                        y.Append(",slipAngle").Append(Wheels[w]).Append('=')
                         .Append(ang[w].ToString("0.###", CultureInfo.InvariantCulture));
            }
            for (var w = 0; w < 4; w++) { _prevPx[w] = px[w]; _prevPz[w] = pz[w]; }
            _hadContact = true;

            // The game's own kerb vibration (Competizione, EVO), unless the
            // settings file binds a curbs property.
            if (_acKerb != null && string.IsNullOrWhiteSpace(_curbsProp))
            {
                try
                {
                    var k = Convert.ToDouble(_acKerb.GetValue(phys), CultureInfo.InvariantCulture) * 100.0;
                    if (!double.IsNaN(k)) y.Append(",curbs=").Append(Math.Max(0, Math.Min(100, k)).ToString("0.#", CultureInfo.InvariantCulture));
                }
                catch { }
            }
        }

        // Automobilista 2: the surface under each tyre (the game's terrain
        // material), mapped onto the protocol's plain classes, and the rain.
        private static void AppendAms2Surface(StringBuilder y, PluginManager pm)
        {
            double rain;
            if (ReadRaw(pm, "DataCorePlugin.GameRawData.mRainDensity", out rain) && !double.IsNaN(rain))
                y.Append(",rain=").Append(Math.Max(0, Math.Min(1, rain)).ToString("0.###", CultureInfo.InvariantCulture));
            var cls = new int[4];
            for (var w = 0; w < 4; w++)
            {
                double t;
                if (!ReadRaw(pm, "DataCorePlugin.GameRawData.mTerrain0" + (w + 1), out t)) return;
                cls[w] = Ams2SurfaceClass((int)t);
            }
            for (var w = 0; w < 4; w++)
                y.Append(",surface").Append(Wheels[w]).Append('=').Append(cls[w].ToString(CultureInfo.InvariantCulture));
        }

        // AMS2 terrain materials (SimHub's AMS2 eTerrain) -> 0 tarmac,
        // 1 bumpy tarmac, 2 kerb (rumble strip), 3 gravel, 4 grass, 5 dirt,
        // 6 cobbles, 7 snow, 8 ice, 9 sand.
        private static int Ams2SurfaceClass(int t)
        {
            switch (t)
            {
                case 2: case 3: case 4: case 11: case 35: case 36: return 1;          // bumpy roads, drains, damaged, train track
                case 10: case 25: case 40: case 41: return 2;                         // rumble strips, exit rumble strips, B1/B2 rumbles
                case 8: case 9: return 3;                                             // gravel, bumpy gravel
                case 6: case 7: case 24: case 26: case 27: case 28: case 32: return 4; // grass, verges, grasscrete, astroturf
                case 5: case 17: case 18: case 19: case 20: case 22: case 31: return 5; // marbles, dirt, dirt roads, dirt bank, baked clay
                case 23: case 29: case 37: return 6;                                  // wood, cobbles
                case 33: case 34: case 44: return 7;                                  // snow half, snow full, snow walls
                case 45: return 8;                                                    // ice road
                case 15: case 16: case 30: case 42: case 43: return 9;                // sand, bumpy sand, sand road, rough sand
                default: return 0;
            }
        }

        private static double Sq(double v) { return v * v; }

        // Reflection over SimHub's Assetto Corsa reader types, cached by type.
        private Type _acRawType, _acPhysType, _acCoordType;
        private FieldInfo _acPhysField, _acContactPoint, _acContactHeading, _acKerb, _acCx, _acCy, _acCz;
        private PropertyInfo _acPhysProp;
        private readonly double[] _prevPx = new double[4], _prevPz = new double[4];
        private bool _hadContact;

        private object AcPhysics(object raw)
        {
            if (raw == null) return null;
            var t = raw.GetType();
            if (t != _acRawType)
            {
                _acRawType   = t;
                _acPhysField = t.GetField("Physics", BindingFlags.Public | BindingFlags.Instance);
                _acPhysProp  = _acPhysField == null ? t.GetProperty("Physics", BindingFlags.Public | BindingFlags.Instance) : null;
                _hadContact  = false;
            }
            object ph = null;
            try { ph = _acPhysField != null ? _acPhysField.GetValue(raw) : (_acPhysProp != null ? _acPhysProp.GetValue(raw, null) : null); }
            catch { ph = null; }
            if (ph == null) return null;
            var pt = ph.GetType();
            if (pt != _acPhysType)
            {
                _acPhysType       = pt;
                _acContactPoint   = pt.GetField("TyreContactPoint");
                _acContactHeading = pt.GetField("TyreContactHeading");
                _acKerb           = pt.GetField("kerbVibration") ?? pt.GetField("KerbVibration");
            }
            return ph;
        }

        private bool AcVec(Array arr, int i, out double x, out double y, out double z)
        {
            x = y = z = 0;
            if (arr == null || i >= arr.Length) return false;
            var e = arr.GetValue(i);
            if (e == null) return false;
            var et = e.GetType();
            if (et != _acCoordType)
            {
                _acCoordType = et;
                _acCx = et.GetField("X"); _acCy = et.GetField("Y"); _acCz = et.GetField("Z");
            }
            if (_acCx == null || _acCy == null || _acCz == null) return false;
            try
            {
                x = Convert.ToDouble(_acCx.GetValue(e), CultureInfo.InvariantCulture);
                y = Convert.ToDouble(_acCy.GetValue(e), CultureInfo.InvariantCulture);
                z = Convert.ToDouble(_acCz.GetValue(e), CultureInfo.InvariantCulture);
            }
            catch { return false; }
            return !(double.IsNaN(x) || double.IsNaN(y) || double.IsNaN(z) || double.IsInfinity(x) || double.IsInfinity(y) || double.IsInfinity(z));
        }

        private void Send(string line)
        {
            try
            {
                var bytes = Encoding.ASCII.GetBytes(line);
                _udp.Send(bytes, bytes.Length, _target);
            }
            catch { /* transient socket errors are not worth a log storm */ }
        }

        // Free text for the identity line: ASCII only, no separators, capped
        // to what the receiver stores.
        private static string Clean(string s)
        {
            if (string.IsNullOrEmpty(s)) return "";
            var sb = new StringBuilder(s.Length);
            foreach (var c in s)
            {
                if (c == ',' || c == '=' || c == '\n' || c == '\r') sb.Append(' ');
                else if (c < 32 || c > 126) sb.Append('?');
                else sb.Append(c);
            }
            var t = sb.ToString().Trim();
            return t.Length > 47 ? t.Substring(0, 47) : t;
        }

        // Optional magnitude channel: any SimHub property name, expected to
        // yield 0..100. Unset, missing, or non-numeric = 0 (channel inert).
        private static double ReadProp(PluginManager pm, string prop)
        {
            double v;
            if (!ReadRaw(pm, prop, out v)) return 0;
            return v < 0 ? 0 : (v > 100 ? 100 : v);
        }

        // Raw numeric property: true when the property exists and is a
        // finite number this tick.
        private static bool ReadRaw(PluginManager pm, string prop, out double value)
        {
            value = 0;
            if (string.IsNullOrWhiteSpace(prop) || pm == null) return false;
            try
            {
                var v = pm.GetPropertyValue(prop.Trim());
                if (v == null) return false;
                var d = Convert.ToDouble(v, CultureInfo.InvariantCulture);
                if (double.IsNaN(d) || double.IsInfinity(d)) return false;
                value = d;
                return true;
            }
            catch { return false; }
        }

        private static string ExtractString(string json, string key)
        {
            var k = "\"" + key + "\"";
            var i = json.IndexOf(k, StringComparison.OrdinalIgnoreCase);
            if (i < 0) return null;
            i = json.IndexOf(':', i + k.Length); if (i < 0) return null;
            var q1 = json.IndexOf('"', i + 1);   if (q1 < 0) return null;
            var q2 = json.IndexOf('"', q1 + 1);  if (q2 < 0) return null;
            return json.Substring(q1 + 1, q2 - q1 - 1);
        }

        private static double ExtractNumber(string json, string key)
        {
            var k = "\"" + key + "\"";
            var i = json.IndexOf(k, StringComparison.OrdinalIgnoreCase);
            if (i < 0) return -1;
            i = json.IndexOf(':', i + k.Length); if (i < 0) return -1;
            var j = i + 1;
            while (j < json.Length && (char.IsWhiteSpace(json[j]))) j++;
            var start = j;
            while (j < json.Length && (char.IsDigit(json[j]) || json[j] == '.' || json[j] == '-')) j++;
            double n;
            return double.TryParse(json.Substring(start, j - start), NumberStyles.Float, CultureInfo.InvariantCulture, out n) ? n : -1;
        }
    }
}
