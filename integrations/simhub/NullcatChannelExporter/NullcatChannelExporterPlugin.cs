// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
//
// nullCAT Channel Exporter - a deliberately dumb SimHub plugin.
//
// Sends up to three UDP lines per data tick (protocol 1.9, Docs/PROTOCOL.md):
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
//   NULLCATY,game=..,car=..,carId=..,direct=..     (once a second; direct =
//            on, off, or none: whether the Kunos direct read below runs)
//
//   NULLCATY,accSurge=..,accSway=..,accHeave=..    (optional, the Assetto
//            Corsa family: the accelerations from the game's shared memory
//            on every physics step, see KunosDirect.cs; the line above then
//            leaves them out)
//
// And it hears one line back (protocol 1.9), only from nullCAT (the
// address it sends to, or nullCAT's telemetry port): NULLCATC,direct=on|off,
// nullCAT's Direct telemetry switch,
// which turns the Kunos direct read on or off.
//
// That is the whole job. No shaping, no state: nullCAT owns the tyre
// model and maps channels onto effects. The only game knowledge here is
// WHICH raw fields each sim exposes per wheel (the built-in presets), so
// a known sim needs no setup at all; the settings file overrides a group
// when it binds all four wheels. A group that is not in force is simply
// absent (never a row of zeros a receiver could mistake for "no slip").
//
// Configuration: NullcatChannelExporter.json, read once at start from the
// first of %LOCALAPPDATA%\nullCAT\, the VirtualStore copy of the file
// beside this DLL, and the file beside this DLL (see SettingsCandidates),
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
        private readonly object _sendLock = new object();   // the fast line sends from its own thread
        private KunosDirect _kunos;
        private string _kunosGame;    // the game the Kunos direct match was made for
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
        // Prefix + 01..04 names a wheel's value; with Field, a struct per
        // wheel (Prefix01.Field, SimHub's naming for an array of structs).
        private struct PresetGroup
        {
            public string Group, Prefix, Field; public double Scale;
            public PresetGroup(string g, string p, double s) { Group = g; Prefix = p; Field = null; Scale = s; }
            public PresetGroup(string g, string p, string f, double s) { Group = g; Prefix = p; Field = f; Scale = s; }
        }
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
            // Competizione: a real slip ratio per wheel, the rest as AC, but
            // no load (the game leaves WheelLoad at zero: not sending it shows
            // "not sent" on the rig instead of a row of zeros).
            new PresetGroup("slipRatio",  "DataCorePlugin.GameRawData.Physics.slipRatio",         1),
            new PresetGroup("wheelSpeed", "DataCorePlugin.GameRawData.Physics.WheelAngularSpeed", 1),
            new PresetGroup("wheelSlip",  "DataCorePlugin.GameRawData.Physics.WheelSlip",         AcSlipScale),
            new PresetGroup("suspTravel", "DataCorePlugin.GameRawData.Physics.SuspensionTravel",  1000),
        };
        private static readonly PresetGroup[] PresetACEvoRally = {
            // EVO and Rally ride on Competizione's layout until each gets its
            // own pass (a real slip ratio per wheel, the rest as AC).
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
        // Le Mans Ultimate (SimHub's rFactor 2 layout of its shared memory):
        // wheel rotation (rad/s), tyre load (N), suspension deflection
        // (m -> mm). The slide, flats, surface, wear and weather go in
        // AppendLmu.
        private const string LmuWheels = "DataCorePlugin.GameRawData.CurrentPlayerTelemetry.mWheels";
        private static readonly PresetGroup[] PresetLMU = {
            new PresetGroup("wheelSpeed", LmuWheels, "mRotation",             1),
            new PresetGroup("load",       LmuWheels, "mTireLoad",             1),
            new PresetGroup("suspTravel", LmuWheels, "mSuspensionDeflection", 1000),
        };
        private static PresetGroup[] PresetFor(string gameName)
        {
            var g = Norm(gameName);
            if (g == "assettocorsa") return PresetAC;
            if (g == "assettocorsacompetizione") return PresetACC;
            if (g == "assettocorsaevo" || g == "assettocorsarally") return PresetACEvoRally;
            if (g == "automobilista2") return PresetAMS2;
            if (g == "lmu") return PresetLMU;
            return null;
        }
        // The Assetto Corsa family: AC and, riding on its shared memory
        // layout until each has its own pass, Competizione, EVO and Rally.
        private static bool IsAcFamily(string family)
        {
            return family == "assettocorsa" || family == "assettocorsacompetizione" || family == "assettocorsaevo" || family == "assettocorsarally";
        }
        private static string Norm(string s)
        {
            if (string.IsNullOrEmpty(s)) return "";
            var sb = new StringBuilder(s.Length);
            foreach (var c in s) if (char.IsLetterOrDigit(c)) sb.Append(char.ToLowerInvariant(c));
            return sb.ToString();
        }
        private string _presetGame;   // the game the groups in force were resolved for
        private string _pitGame;      // the game the pit flag was last judged for...
        private bool   _pitWorks;     // ...and whether it has shown the flag set

        private DateTime _lastIdentity = DateTime.MinValue;

        // Where the settings file is read from; the first that exists wins.
        // SimHub sits under Program Files, where an editor's save without
        // administrator rights is quietly redirected by Windows into a
        // VirtualStore copy the plugin used to ignore, so an edit seemed to
        // vanish. 1: a user-writable folder that SimHub updates never touch;
        // 2: that VirtualStore copy of the file beside the DLL; 3: the file
        // beside the DLL, where a release puts it.
        private static string[] SettingsCandidates(string dllDir)
        {
            const string name = "NullcatChannelExporter.json";
            var list = new System.Collections.Generic.List<string>();
            var local = Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData);
            if (!string.IsNullOrEmpty(local))
            {
                list.Add(Path.Combine(local, "nullCAT", name));
                if (!string.IsNullOrEmpty(dllDir))
                {
                    // Windows keeps VirtualStore copies for the system drive's
                    // protected folders only (a UNC root has no trailing slash,
                    // so the rest would read as rooted and escape the folder).
                    var root    = Path.GetPathRoot(dllDir) ?? "";
                    var sysRoot = Path.GetPathRoot(Environment.GetFolderPath(Environment.SpecialFolder.Windows)) ?? "";
                    if (root.Length > 0 && dllDir.Length > root.Length
                        && string.Equals(root, sysRoot, StringComparison.OrdinalIgnoreCase))
                        list.Add(Path.Combine(local, "VirtualStore", dllDir.Substring(root.Length).TrimStart('\\', '/'), name));
                }
            }
            list.Add(Path.Combine(dllDir ?? ".", name));
            return list.ToArray();
        }

        public void Init(PluginManager pluginManager)
        {
            _pm = pluginManager;
            var host = "127.0.0.1";
            var port = 4444;
            var settingsNote = "no NullcatChannelExporter.json found, defaults";
            var kunosOn = false;
            var heightsOn = false;
            for (var g = 0; g < Groups.Length; g++) { _fileProps[g] = new string[4]; _groupProps[g] = new string[4]; }
            try
            {
                var dir  = Path.GetDirectoryName(typeof(NullcatChannelExporterPlugin).Assembly.Location);
                var candidates = SettingsCandidates(dir);
                string path = null;
                foreach (var c in candidates) if (File.Exists(c)) { path = c; break; }
                if (path == null)
                    settingsNote = "no NullcatChannelExporter.json found (looked in " + string.Join(", ", candidates) + "), defaults";
                else
                {
                    settingsNote = "settings from " + path;
                    // A copy found earlier hides the later ones: say which, so
                    // a stale copy in the way is easy to spot and delete.
                    var skipped = new System.Collections.Generic.List<string>();
                    foreach (var c in candidates) if (c != path && File.Exists(c)) skipped.Add(c);
                    if (skipped.Count > 0) settingsNote += " (also present, not read: " + string.Join(", ", skipped) + ")";
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
                    var kd = ExtractString(text, "kunosDirect");
                    kunosOn = kd != null && (kd.Trim().Equals("on", StringComparison.OrdinalIgnoreCase) || kd.Trim().Equals("true", StringComparison.OrdinalIgnoreCase));
                    var kh = ExtractString(text, "kunosDirectHeights");
                    heightsOn = kh != null && (kh.Trim().Equals("on", StringComparison.OrdinalIgnoreCase) || kh.Trim().Equals("true", StringComparison.OrdinalIgnoreCase));
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
            // Bound at once (an ephemeral port) so the listener below can
            // hear nullCAT's answer on the socket the lines leave from: the
            // firewall lets a reply to them through.
            _udp = new UdpClient(new IPEndPoint(IPAddress.Any, 0));
            // Windows reports a send to a port nobody listened on (nullCAT
            // not running yet) as an error on the NEXT receive: off.
            try { const int SIO_UDP_CONNRESET = -1744830452; _udp.Client.IOControl(SIO_UDP_CONNRESET, new byte[] { 0 }, null); } catch { }
            new System.Threading.Thread(Listen) { IsBackground = true, Name = "nullCAT answers" }.Start();

            // Say where the stream goes, in SimHub's log and as a property
            // anyone can read in the property list: when nothing arrives at
            // the rig, this is the first thing to look at.
            _targetText = host + ":" + port;
            try { SimHub.Logging.Current.Info("nullCAT Channel Exporter: sending to " + _targetText + " (" + settingsNote + ")"); } catch { }
            try { pluginManager.AddProperty("Target", GetType(), _targetText); } catch { }
            try { pluginManager.AddProperty("Game", GetType(), ""); } catch { }

            // The Assetto Corsa family's accelerations from the game's shared
            // memory (KunosDirect.cs): from the settings file, and flipped
            // while driving by the ToggleKunosDirect action (bind it to a
            // button under Controls and events) to compare it with SimHub's.
            _kunos = new KunosDirect(kunosOn, heightsOn, string.IsNullOrWhiteSpace(_curbsProp), Send, msg => { try { SimHub.Logging.Current.Info(msg); } catch { } });
            try { pluginManager.AddProperty("KunosDirect", GetType(), _kunos.Status()); } catch { }
            try
            {
                pluginManager.AddAction("ToggleKunosDirect", GetType(),
                    (pm, a) => { var k = _kunos; if (k != null) k.Enabled = !k.Enabled; }, (pm, a) => { });
            }
            catch { }
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
                    for (var w = 0; w < 4; w++) _groupProps[g][w] = pg.Prefix + "0" + (w + 1) + (pg.Field != null ? "." + pg.Field : "");
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
            // Protocol 1.9: the pit flag and the lap, for nullCAT's fresh
            // tyres on a return to the pits or a restart. The pit flag from
            // the game's own field where the plugin knows it, from the start:
            // Automobilista 2's pit mode (in the pit box or the garage;
            // driving in or out is not) and the Assetto Corsa family's
            // in-pit flag. For other games SimHub's, only once the game has
            // shown it set: a game SimHub cannot read it for would send a
            // steady 0, which reads as "on track" and turns nullCAT's
            // fallback off.
            var family = Norm(data.GameName);
            if (family != _pitGame) { _pitGame = family; _pitWorks = false; }
            var kunos = _kunos;   // End may clear the field meanwhile
            if (kunos != null)
            {
                kunos.SetGame(IsAcFamily(family), family != _kunosGame, family != "assettocorsa");
                _kunosGame = family;
                double? surge = null, sway = null, heaveAcc = null;
                try { surge = d.AccelerationSurge; sway = d.AccelerationSway; heaveAcc = d.AccelerationHeave; } catch { }
                kunos.Observe(surge, sway, heaveAcc, d.SpeedKmh);
            }
            double inPit = 0;
            try { inPit = Convert.ToDouble(d.IsInPit, CultureInfo.InvariantCulture) > 0 ? 1 : 0; } catch { inPit = 0; }
            var pitKnown = false;
            double pitRaw;
            if (family == "automobilista2" && ReadRaw(pluginManager, "DataCorePlugin.GameRawData.mPitMode", out pitRaw))
            {
                pitKnown = true;
                inPit = (pitRaw == 2 || pitRaw == 4) ? 1 : 0;
            }
            else if (IsAcFamily(family) && ReadRaw(pluginManager, "DataCorePlugin.GameRawData.Graphics.IsInPit", out pitRaw))
            {
                pitKnown = true;
                inPit = pitRaw > 0 ? 1 : 0;
            }
            else if (family == "lmu")
            {
                // In the garage stall, or stopped in the pit box (pit state
                // 3); the game's mInPits is the whole pit lane.
                double stall, state;
                if (ReadRaw(pluginManager, "DataCorePlugin.GameRawData.CurrentPlayer.mInGarageStall", out stall)
                    && ReadRaw(pluginManager, "DataCorePlugin.GameRawData.CurrentPlayer.mPitState", out state))
                {
                    pitKnown = true;
                    inPit = (stall > 0 || state == 3) ? 1 : 0;
                }
            }
            if (inPit > 0) _pitWorks = true;
            if (pitKnown || _pitWorks) y.Append(",inPit=").Append(inPit.ToString("0", CultureInfo.InvariantCulture));
            try
            {
                var lap = Convert.ToDouble(d.CurrentLap, CultureInfo.InvariantCulture);
                if (!double.IsNaN(lap) && !double.IsInfinity(lap)) y.Append(",lap=").Append(lap.ToString("0", CultureInfo.InvariantCulture));
            }
            catch { }
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
            // What the fast line carries is left out here (one source per
            // channel: two would alternate on the rig).
            var fastSurge = kunos != null && kunos.Carries(0);
            var fastSway  = kunos != null && kunos.Carries(1);
            var fastHeave = kunos != null && kunos.Carries(2);
            AppendChassis(y, d, !fastHeave);
            AppendBodyMotion(y, d, !fastSurge, !fastSway);
            if (IsAcFamily(family))
            {
                AppendAcContact(y, d, family, kunos != null && kunos.Carries(3), kunos != null && kunos.Carries(4));
                if (family == "assettocorsacompetizione") AppendAccWeather(y, pluginManager);
            }
            else if (family == "automobilista2")
            {
                AppendAms2Surface(y, pluginManager);
                AppendAms2SlipSpeed(y, pluginManager);
                AppendAms2TyreWear(y, pluginManager);
            }
            else if (family == "lmu")
                AppendLmu(y, pluginManager);
            Send(y.ToString());
            if (kunos != null) kunos.MainLineSent();

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
                // direct: whether the Kunos direct read runs for this game
                // (none: the game has no direct read), for nullCAT's Direct
                // telemetry switch.
                var direct = kunos == null || !kunos.GameOk ? "none" : kunos.Enabled ? "on" : "off";
                if (game.Length > 0 || car.Length > 0)
                    Send("NULLCATY,game=" + game + ",car=" + car + ",carId=" + carId + ",direct=" + direct);
                if (kunos != null)
                {
                    kunos.Tick();
                    try { pluginManager.SetPropertyValue("KunosDirect", GetType(), kunos.Status()); } catch { }
                }
            }
        }

        public void End(PluginManager pluginManager)
        {
            var kunos = _kunos;
            _kunos = null;
            kunos?.Dispose();
            lock (_sendLock)
            {
                _udp?.Close();
                _udp = null;
            }
        }

        // ---- protocol 1.5: the body and the ground --------------------------

        // The body's vertical acceleration, pitch and roll, from SimHub's own
        // normalized motion data (the same values its motion output uses, so
        // they mean the same thing in every game). Heave is already in m/s^2
        // there (read as G, a parked car shook at 0.7 g and a lap averaged
        // 2.6 g; as m/s^2, 0.07 and 0.27 g): sent as it is.
        // Protocol 1.9: the rest of the body's motion as SimHub gives it, each
        // only where the game has it (SimHub leaves them empty otherwise):
        // the accelerations along and across the car (the same unit as
        // accHeave) and the rates the car turns at in yaw, pitch and roll.
        // For nullCAT's belt and haptics; the motion cue stays SimHub's.
        private static void AppendBodyMotion(StringBuilder y, StatusDataBase d, bool surge, bool sway)
        {
            if (surge) AppendIfKnown(y, "accSurge",  () => d.AccelerationSurge);
            if (sway)  AppendIfKnown(y, "accSway",   () => d.AccelerationSway);
            AppendIfKnown(y, "yawRate",   () => d.YawChangeVelocity);
            AppendIfKnown(y, "pitchRate", () => d.PitchChangeVelocity);
            AppendIfKnown(y, "rollRate",  () => d.RollChangeVelocity);
        }
        private static void AppendIfKnown(StringBuilder y, string token, Func<double?> read)
        {
            double? v;
            try { v = read(); } catch { return; }
            if (!v.HasValue || double.IsNaN(v.Value) || double.IsInfinity(v.Value)) return;
            y.Append(',').Append(token).Append('=').Append(v.Value.ToString("0.####", CultureInfo.InvariantCulture));
        }

        private static void AppendChassis(StringBuilder y, StatusDataBase d, bool sendHeave)
        {
            double? heave = null;
            try { heave = d.AccelerationHeave; } catch { heave = null; }
            if (!heave.HasValue || double.IsNaN(heave.Value) || double.IsInfinity(heave.Value)) return;
            double pitch, roll;
            try { pitch = d.OrientationPitch; roll = d.OrientationRoll; } catch { return; }
            if (double.IsNaN(pitch) || double.IsNaN(roll)) return;
            if (sendHeave) y.Append(",accHeave=").Append(heave.Value.ToString("0.###", CultureInfo.InvariantCulture));
            y.Append(",pitchDeg=").Append(pitch.ToString("0.####", CultureInfo.InvariantCulture))
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
        // fastHeights / fastKerb: the direct read's fast line carries them now
        // (one source per channel: two would alternate on the rig).
        private void AppendAcContact(StringBuilder y, StatusDataBase d, string game, bool fastHeights, bool fastKerb)
        {
            object raw;
            try { raw = d.GetRawDataObject(); } catch { raw = null; }
            var phys = AcPhysics(raw);
            AcDiagnose(game, raw, phys);
            if (phys == null) return;
            var fileSlip = !string.IsNullOrWhiteSpace(_fileProps[0][0]);
            if (!AppendAcPatches(y, d, phys, fastHeights, fileSlip))
            {
                _hadContact = false;
                // No contact points from this game: its own slip angles
                // where its page has them (Competizione, EVO, Rally), in
                // radians on the page.
                if (!fileSlip && _acSlipAngle != null && d.SpeedKmh > 5.0)
                {
                    Array sa = null;
                    try { sa = _acSlipAngle.GetValue(phys) as Array; } catch { sa = null; }
                    if (sa != null && sa.Length >= 4)
                    {
                        var ang = new double[4]; var all = true;
                        for (var w = 0; w < 4 && all; w++)
                        {
                            try { ang[w] = Convert.ToDouble(sa.GetValue(w), CultureInfo.InvariantCulture) * 180.0 / Math.PI; }
                            catch { all = false; }
                            if (double.IsNaN(ang[w]) || double.IsInfinity(ang[w])) all = false;
                        }
                        if (all)
                            for (var w = 0; w < 4; w++)
                                y.Append(",slipAngle").Append(Wheels[w]).Append('=')
                                 .Append(ang[w].ToString("0.###", CultureInfo.InvariantCulture));
                    }
                }
            }

            // The game's own kerb vibration (Competizione, EVO, Rally), unless
            // the settings file binds a curbs property or the fast line
            // carries it. Never behind the contact points (a game without
            // them used to lose its kerbs too).
            if (_acKerb != null && string.IsNullOrWhiteSpace(_curbsProp) && !fastKerb)
            {
                try
                {
                    var k = Convert.ToDouble(_acKerb.GetValue(phys), CultureInfo.InvariantCulture) * 100.0;
                    if (!double.IsNaN(k)) y.Append(",curbs=").Append(Math.Max(0, Math.Min(100, k)).ToString("0.#", CultureInfo.InvariantCulture));
                }
                catch { }
            }
        }

        // The contact patches: road heights (unless the fast line carries
        // them), wheelbase and track, and a slip angle per wheel from where
        // each patch moved. False when the game gives no usable patches.
        private bool AppendAcPatches(StringBuilder y, StatusDataBase d, object phys, bool fastHeights, bool fileSlip)
        {
            if (_acContactPoint == null) return false;
            var cp = _acContactPoint.GetValue(phys) as Array;
            var ch = _acContactHeading != null ? _acContactHeading.GetValue(phys) as Array : null;
            var px = new double[4]; var py = new double[4]; var pz = new double[4];
            for (var w = 0; w < 4; w++)
                if (!AcVec(cp, w, out px[w], out py[w], out pz[w])) return false;
            // All four at the origin: a page that does not fill them.
            if (px[0] == 0 && py[0] == 0 && pz[0] == 0 && px[3] == 0 && py[3] == 0 && pz[3] == 0) return false;

            // Road height under each tyre, mm (hills and all: nullCAT filters).
            if (!fastHeights)
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
            return true;
        }

        // Once per game and reader type, SimHub's log says what the game's
        // raw data offers this plugin: a game whose channels go missing (AC
        // EVO's kerbs, 2026-10) is diagnosed from the log alone.
        private string _acDiagKey = "";
        private void AcDiagnose(string game, object raw, object phys)
        {
            var key = game + "|" + (raw == null ? "-" : raw.GetType().FullName) + "|" + (phys == null ? "-" : phys.GetType().FullName);
            if (key == _acDiagKey) return;
            _acDiagKey = key;
            string sample = "";
            if (phys != null && _acContactPoint != null)
            {
                try
                {
                    var cp = _acContactPoint.GetValue(phys) as Array;
                    double x, yy, z;
                    sample = cp == null ? " (contact points: no array)"
                           : AcVec(cp, 0, out x, out yy, out z) ? " (front left patch " + x.ToString("0.###", CultureInfo.InvariantCulture) + ", "
                               + yy.ToString("0.###", CultureInfo.InvariantCulture) + ", " + z.ToString("0.###", CultureInfo.InvariantCulture) + ")"
                           : " (contact points unreadable: no X/Y/Z or not a number)";
                }
                catch (Exception ex) { sample = " (contact points: " + ex.Message + ")"; }
            }
            var msg = "nullCAT Channel Exporter: " + game + " raw data: "
                    + (raw == null ? "none" : raw.GetType().Name)
                    + (phys == null ? ", no Physics" : ", Physics " + phys.GetType().FullName)
                    + ", contact points " + (_acContactPoint != null ? "yes" : "no")
                    + ", headings " + (_acContactHeading != null ? "yes" : "no")
                    + ", kerb vibration " + (_acKerb != null ? "yes" : "no")
                    + ", slip angles " + (_acSlipAngle != null ? "yes" : "no") + sample;
            try { SimHub.Logging.Current.Info(msg); } catch { }
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
            AppendAms2TyreFlags(y, pm);
        }

        // Automobilista 2 tyre slip speed as it is, m/s (protocol 1.8): the
        // contact patch slipping over the road, sideways and along together.
        // nullCAT takes the along part out with the wheel speeds and plays
        // the rest as a slip angle (the plain combined slip grew with road
        // speed and read as a slide on every straight).
        private static void AppendAms2SlipSpeed(StringBuilder y, PluginManager pm)
        {
            var ss = new double[4];
            for (var w = 0; w < 4; w++)
                if (!ReadRaw(pm, "DataCorePlugin.GameRawData.mTyreSlipSpeed0" + (w + 1), out ss[w]) || double.IsNaN(ss[w]) || double.IsInfinity(ss[w])) return;
            for (var w = 0; w < 4; w++)
                y.Append(",slipSpeed").Append(Wheels[w]).Append('=').Append(Math.Abs(ss[w]).ToString("0.###", CultureInfo.InvariantCulture));
        }

        // Competizione's weather (protocol 1.6 tokens): the rain from the
        // game's rain intensity (0 none .. 5 thunderstorm, as 0..1), and the
        // water on the road from its track grip status (0 green, 1 fast and
        // 2 optimum are dry; 3 greasy, 4 damp, 5 wet, 6 flooded), a starting
        // scale for the Surface and Road tiles.
        private static readonly double[] AccGripWet = { 0, 0, 0, 0.15, 0.35, 0.7, 1.0 };
        private static void AppendAccWeather(StringBuilder y, PluginManager pm)
        {
            double rain, grip;
            if (ReadRaw(pm, "DataCorePlugin.GameRawData.Graphics.rainIntensity", out rain))
                y.Append(",rain=").Append(Math.Max(0.0, Math.Min(1.0, rain / 5.0)).ToString("0.###", CultureInfo.InvariantCulture));
            if (ReadRaw(pm, "DataCorePlugin.GameRawData.Graphics.trackGripStatus", out grip))
            {
                var i = (int)Math.Round(grip);
                if (i >= 0 && i < AccGripWet.Length)
                    y.Append(",wet=").Append(AccGripWet[i].ToString("0.###", CultureInfo.InvariantCulture));
            }
        }

        // Le Mans Ultimate's per-wheel extras and weather: the contact patch
        // sliding over the road (the lateral and longitudinal patch speeds
        // together, m/s), a flat tyre, the surface (the game's 0 dry, 1 wet,
        // 2 grass, 3 dirt, 4 gravel, 5 kerb, 6 special), the wear (the
        // game's 1 new .. 0 worn, sent as worn), the rain and the track's
        // average wetness (both 0..1). Each group only when all four wheels
        // read.
        private static readonly int[] LmuSurfaceClass = { 0, 0, 4, 5, 3, 2, 0 };
        private static bool ReadWheels(PluginManager pm, string field, double[] into)
        {
            for (var w = 0; w < 4; w++)
                if (!ReadRaw(pm, LmuWheels + "0" + (w + 1) + "." + field, out into[w])) return false;
            return true;
        }
        private static void AppendLmu(StringBuilder y, PluginManager pm)
        {
            var a = new double[4]; var b = new double[4];
            if (ReadWheels(pm, "mLateralPatchVel", a) && ReadWheels(pm, "mLongitudinalPatchVel", b))
                for (var w = 0; w < 4; w++)
                    y.Append(",slipSpeed").Append(Wheels[w]).Append('=')
                     .Append(Math.Sqrt(a[w] * a[w] + b[w] * b[w]).ToString("0.###", CultureInfo.InvariantCulture));
            if (ReadWheels(pm, "mFlat", a))
                for (var w = 0; w < 4; w++) y.Append(",deflated").Append(Wheels[w]).Append('=').Append(a[w] > 0 ? '1' : '0');
            if (ReadWheels(pm, "mSurfaceType", a))
                for (var w = 0; w < 4; w++)
                {
                    var t = (int)Math.Round(a[w]);
                    var cls = (t >= 0 && t < LmuSurfaceClass.Length) ? LmuSurfaceClass[t] : 0;
                    y.Append(",surface").Append(Wheels[w]).Append('=').Append(cls.ToString(CultureInfo.InvariantCulture));
                }
            if (ReadWheels(pm, "mWear", a))
                for (var w = 0; w < 4; w++)
                    y.Append(",tyreWear").Append(Wheels[w]).Append('=')
                     .Append(Math.Max(0.0, Math.Min(1.0, 1.0 - a[w])).ToString("0.####", CultureInfo.InvariantCulture));
            double rain, wet;
            if (ReadRaw(pm, "DataCorePlugin.GameRawData.Data.mRaining", out rain))
                y.Append(",rain=").Append(Math.Max(0.0, Math.Min(1.0, rain)).ToString("0.###", CultureInfo.InvariantCulture));
            if (ReadRaw(pm, "DataCorePlugin.GameRawData.Data.mAvgPathWetness", out wet))
                y.Append(",wet=").Append(Math.Max(0.0, Math.Min(1.0, wet)).ToString("0.###", CultureInfo.InvariantCulture));
        }

        // Automobilista 2 tyre wear (protocol 1.9): 0 new .. 1 worn out, as
        // the game gives it. nullCAT reads a sudden fall as new tyres.
        private static void AppendAms2TyreWear(StringBuilder y, PluginManager pm)
        {
            var wear = new double[4];
            for (var w = 0; w < 4; w++)
                if (!ReadRaw(pm, "DataCorePlugin.GameRawData.mTyreWear0" + (w + 1), out wear[w])) return;
            for (var w = 0; w < 4; w++)
                y.Append(",tyreWear").Append(Wheels[w]).Append('=')
                 .Append(Math.Max(0.0, Math.Min(1.0, wear[w])).ToString("0.####", CultureInfo.InvariantCulture));
        }

        // Automobilista 2 tyre flags (protocol 1.7): bit 0 attached, bit 1
        // inflated, bit 2 on the ground. A tyre still attached but no longer
        // inflated is punctured.
        private static void AppendAms2TyreFlags(StringBuilder y, PluginManager pm)
        {
            var flags = new int[4];
            for (var w = 0; w < 4; w++)
            {
                double f;
                if (!ReadRaw(pm, "DataCorePlugin.GameRawData.mTyreFlags0" + (w + 1), out f) || double.IsNaN(f)) return;
                flags[w] = (int)f;
            }
            for (var w = 0; w < 4; w++)
                y.Append(",onGround").Append(Wheels[w]).Append('=').Append((flags[w] & 4) != 0 ? '1' : '0');
            for (var w = 0; w < 4; w++)
                y.Append(",deflated").Append(Wheels[w]).Append('=').Append(((flags[w] & 1) != 0 && (flags[w] & 2) == 0) ? '1' : '0');
        }

        // AMS2 terrain materials (SimHub's AMS2 eTerrain) -> 0 tarmac,
        // 1 bumpy tarmac, 2 kerb (rumble strip), 3 gravel, 4 grass, 5 dirt,
        // 6 cobbles, 7 snow, 8 ice, 9 sand.
        private static int Ams2SurfaceClass(int t)
        {
            switch (t)
            {
                case 2: case 3: case 4: case 5: case 11: case 35: case 36: return 1;  // bumpy roads, marbles (loose rubber off line), drains, damaged, train track
                case 10: case 25: case 40: case 41: return 2;                         // rumble strips, exit rumble strips, B1/B2 rumbles
                case 8: case 9: return 3;                                             // gravel, bumpy gravel
                case 6: case 7: case 24: case 26: case 27: case 28: case 32: return 4; // grass, verges, grasscrete, astroturf
                case 17: case 18: case 19: case 20: case 22: case 31: return 5;       // dirt, dirt roads, dirt bank, baked clay
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
        private FieldInfo _acPhysField, _acContactPoint, _acContactHeading, _acKerb, _acSlipAngle, _acCx, _acCy, _acCz;
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
                // Names as SimHub's readers spell them, case aside (a
                // reader update that changes the case keeps working).
                const BindingFlags F = BindingFlags.Public | BindingFlags.Instance | BindingFlags.IgnoreCase;
                _acContactPoint   = pt.GetField("TyreContactPoint", F);
                _acContactHeading = pt.GetField("TyreContactHeading", F);
                _acKerb           = pt.GetField("kerbVibration", F);
                _acSlipAngle      = pt.GetField("slipAngle", F);
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

        // nullCAT's answers (protocol 1.9), on the sending socket, only from
        // the address the lines go to: NULLCATC,direct=on|off turns the Kunos
        // direct read on or off (nullCAT's Direct telemetry switch). Runs
        // until End closes the socket.
        private void Listen()
        {
            while (true)
            {
                UdpClient udp;
                lock (_sendLock) udp = _udp;
                if (udp == null) return;
                try
                {
                    var from = new IPEndPoint(IPAddress.Any, 0);
                    var bytes = udp.Receive(ref from);
                    // From nullCAT only: the address the lines go to, or its
                    // telemetry port (a Pi with several addresses may answer
                    // from another one).
                    var target = _target;
                    if (target == null || (!from.Address.Equals(target.Address) && from.Port != target.Port)) continue;
                    var text = Encoding.ASCII.GetString(bytes).Trim();
                    if (!text.StartsWith("NULLCATC,", StringComparison.Ordinal)) continue;
                    foreach (var field in text.Substring(9).Split(','))
                    {
                        var kv = field.Split('=');
                        if (kv.Length != 2 || kv[0].Trim() != "direct") continue;
                        var v = kv[1].Trim();
                        var k = _kunos;
                        if (k == null || (v != "on" && v != "off")) continue;
                        var on = v == "on";
                        if (k.Enabled != on)
                        {
                            k.Enabled = on;
                            try { SimHub.Logging.Current.Info("nullCAT Channel Exporter: direct telemetry " + v + " (asked by nullCAT)"); } catch { }
                        }
                    }
                }
                catch (ObjectDisposedException) { return; }
                catch (SocketException)
                {
                    lock (_sendLock) { if (_udp == null) return; }
                    System.Threading.Thread.Sleep(200);
                }
                catch { System.Threading.Thread.Sleep(200); }
            }
        }

        private void Send(string line)
        {
            try
            {
                var bytes = Encoding.ASCII.GetBytes(line);
                lock (_sendLock) { if (_udp != null) _udp.Send(bytes, bytes.Length, _target); }
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
