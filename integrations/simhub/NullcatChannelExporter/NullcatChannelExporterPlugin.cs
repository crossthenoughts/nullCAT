// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
//
// nullCAT Channel Exporter - a deliberately dumb SimHub plugin.
//
// Sends up to three UDP lines per data tick (protocol 1.3, Docs/PROTOCOL.md):
//
//   NULLCATX,<rpm>,<speedKmh>,<gear>,<clutchPct>,<throttlePct>,
//            <brakePct>,<absActive>,<skid>,<lockup>,<roadNoise>,
//            <limiter>,<tcActive>,<curbs>,<maxRpm>
//
//   NULLCATY,boost=..,pitLimiter=..,slipAngleFL=..,...   (boost and pit limiter always; the
//            per-wheel groups only when bound in the settings file, by name)
//
//   NULLCATY,game=..,car=..                         (once a second)
//
// That is the whole job. No shaping, no game-specific logic, no state:
// nullCAT owns all of that (it holds the tyre model and maps channels onto
// effects). The classic channels go positionally for compatibility with
// older receivers; the per-wheel groups go by NAME so a group that is not
// bound is simply absent (never a row of zeros a receiver could mistake
// for "no slip").
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
        private static readonly string[] Groups = { "slipAngle", "slipRatio", "wheelSpeed", "load", "suspVel" };
        private readonly string[][] _groupProps = new string[5][];
        private readonly double[]   _groupScale = { 1, 1, 1, 1, 1 };

        private DateTime _lastIdentity = DateTime.MinValue;

        public void Init(PluginManager pluginManager)
        {
            var host = "127.0.0.1";
            var port = 4444;
            for (var g = 0; g < Groups.Length; g++) _groupProps[g] = new string[4];
            try
            {
                var dir  = Path.GetDirectoryName(typeof(NullcatChannelExporterPlugin).Assembly.Location);
                var path = Path.Combine(dir ?? ".", "NullcatChannelExporter.json");
                if (File.Exists(path))
                {
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
                            _groupProps[g][w] = ExtractString(text, Groups[g] + Wheels[w] + "Prop");
                        var sc = ExtractNumber(text, Groups[g] + "Scale");
                        if (sc > 0) _groupScale[g] = sc;
                    }
                }
            }
            catch { /* keep defaults */ }

            _target = new IPEndPoint(IPAddress.Parse(host), port);
            _udp = new UdpClient();
        }

        public void DataUpdate(PluginManager pluginManager, ref GameData data)
        {
            if (_udp == null || !data.GameRunning || data.NewData == null) return;
            // A paused game keeps reporting its last frame; sending it would
            // hold an rpm vibration on through the pause menu. Stay silent
            // and nullCAT's staleness fail-safe releases every effect.
            try { if (data.GamePaused) return; } catch { }
            var d = data.NewData;

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

            var line = string.Format(CultureInfo.InvariantCulture,
                "NULLCATX,{0:0.#},{1:0.##},{2:0},{3:0.#},{4:0.#},{5:0.#},{6:0},{7:0.#},{8:0.#},{9:0.#},{10:0},{11:0},{12:0.#},{13:0}",
                d.Rpms, d.SpeedKmh, gear, d.Clutch, d.Throttle,
                d.Brake, abs,
                ReadProp(pluginManager, _skidProp),
                ReadProp(pluginManager, _lockupProp),
                ReadProp(pluginManager, _roadProp),
                limiter, tc,
                ReadProp(pluginManager, _curbsProp),
                maxRpm);
            Send(line);

            // Per-wheel groups by name: only groups with all four wheels
            // bound, only when every wheel reads as a number this tick.
            // Named line: boost every tick (its slot sits past the per-wheel
            // groups, so it cannot go positionally without padding them), then
            // the bound per-wheel groups.
            var y = new StringBuilder("NULLCATY,boost=");
            y.Append(boost.ToString("0.###", CultureInfo.InvariantCulture));
            // Pit limiter: cuts like the rev limiter on the rig, never teaches the redline.
            double pit = 0;
            try { pit = Convert.ToDouble(d.PitLimiterOn, CultureInfo.InvariantCulture) > 0 ? 1 : 0; } catch { pit = 0; }
            y.Append(",pitLimiter=").Append(pit.ToString("0", CultureInfo.InvariantCulture));
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
            Send(y.ToString());

            // Identity once a second: which game and car the channels
            // describe (free text; commas and '=' would break the line).
            var now = DateTime.UtcNow;
            if ((now - _lastIdentity).TotalSeconds >= 1.0)
            {
                _lastIdentity = now;
                var game = Clean(data.GameName);
                var car  = Clean(d.CarModel);
                if (game.Length > 0 || car.Length > 0)
                    Send("NULLCATY,game=" + game + ",car=" + car);
            }
        }

        public void End(PluginManager pluginManager)
        {
            _udp?.Close();
            _udp = null;
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
