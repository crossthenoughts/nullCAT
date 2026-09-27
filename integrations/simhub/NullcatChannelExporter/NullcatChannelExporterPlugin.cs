// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
//
// nullCAT Channel Exporter - a deliberately dumb SimHub plugin.
//
// Sends one UDP line per data tick (protocol 1.1, Docs/PROTOCOL.md):
//   NULLCATX,<rpm>,<speedKmh>,<gear>,<clutchPct>,<throttlePct>,
//            <brakePct>,<absActive>,<skid>,<lockup>,<roadNoise>
//
// That is the whole job. No shaping, no game-specific logic, no state:
// nullCAT owns all of that (the rig's ncxBindings config maps these
// channels onto its effects). Channel order matches the protocol's token
// registry; a channel the current game cannot feed sends 0, which leaves
// its effect silently inert on the rig.
//
// Configuration: NullcatChannelExporter.json next to this DLL,
//   { "host": "192.168.1.50", "port": 4444,
//     "skidProp": "", "lockupProp": "", "roadProp": "" }
// Defaults to 127.0.0.1:4444 when the file is absent or unreadable.
// The three *Prop keys are OPTIONAL full SimHub property names (any
// property, including NCalc-computed ones) expected to yield 0..100;
// empty or missing = that channel sends 0. This is where per-game
// adaptation lives - bind whatever the game exposes for slip, lockup,
// and road surface, and the wire stays semantically clean.

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
    [PluginDescription("Sends raw telemetry channels (rpm, speed, gear, clutch, throttle) to nullCAT as NULLCATX UDP lines")]
    [PluginAuthor("nullCAT")]
    [PluginName("nullCAT Channel Exporter")]
    public class NullcatChannelExporterPlugin : IPlugin, IDataPlugin
    {
        public PluginManager PluginManager { get; set; }

        private UdpClient _udp;
        private IPEndPoint _target;
        private string _skidProp, _lockupProp, _roadProp;

        public void Init(PluginManager pluginManager)
        {
            var host = "127.0.0.1";
            var port = 4444;
            try
            {
                var dir  = Path.GetDirectoryName(typeof(NullcatChannelExporterPlugin).Assembly.Location);
                var path = Path.Combine(dir ?? ".", "NullcatChannelExporter.json");
                if (File.Exists(path))
                {
                    // Tiny hand parser: two known keys, no JSON library needed.
                    var text = File.ReadAllText(path);
                    var h = ExtractString(text, "host");
                    if (!string.IsNullOrWhiteSpace(h)) host = h.Trim();
                    var p = ExtractNumber(text, "port");
                    if (p > 0 && p < 65536) port = p;
                    _skidProp   = ExtractString(text, "skidProp");
                    _lockupProp = ExtractString(text, "lockupProp");
                    _roadProp   = ExtractString(text, "roadProp");
                }
            }
            catch { /* keep defaults */ }

            _target = new IPEndPoint(IPAddress.Parse(host), port);
            _udp = new UdpClient();
        }

        public void DataUpdate(PluginManager pluginManager, ref GameData data)
        {
            if (_udp == null || !data.GameRunning || data.NewData == null) return;
            var d = data.NewData;

            // Gear arrives as a string ("N", "R", "1".."8"); the wire wants a
            // number: N -> 0, R -> -1, digits as-is, anything odd -> 0.
            double gear = 0;
            var g = d.Gear;
            if (!string.IsNullOrEmpty(g))
            {
                if (g == "R" || g == "r") gear = -1;
                else double.TryParse(g, NumberStyles.Integer, CultureInfo.InvariantCulture, out gear);
            }

            // ABS flag: StatusDataBase carries it where the game reports it;
            // anything nonzero on the wire means "cycling".
            double abs = 0;
            try { abs = Convert.ToDouble(d.ABSActive, CultureInfo.InvariantCulture) > 0 ? 1 : 0; }
            catch { abs = 0; }

            var line = string.Format(CultureInfo.InvariantCulture,
                "NULLCATX,{0:0.#},{1:0.##},{2:0},{3:0.#},{4:0.#},{5:0.#},{6:0},{7:0.#},{8:0.#},{9:0.#}",
                d.Rpms, d.SpeedKmh, gear, d.Clutch, d.Throttle,
                d.Brake, abs,
                ReadProp(pluginManager, _skidProp),
                ReadProp(pluginManager, _lockupProp),
                ReadProp(pluginManager, _roadProp));

            try
            {
                var bytes = Encoding.ASCII.GetBytes(line);
                _udp.Send(bytes, bytes.Length, _target);
            }
            catch { /* transient socket errors are not worth a log storm */ }
        }

        public void End(PluginManager pluginManager)
        {
            _udp?.Close();
            _udp = null;
        }

        // Optional property channel: any SimHub property name, expected to
        // yield 0..100. Unset, missing, or non-numeric = 0 (channel inert).
        private static double ReadProp(PluginManager pm, string prop)
        {
            if (string.IsNullOrWhiteSpace(prop) || pm == null) return 0;
            try
            {
                var v = pm.GetPropertyValue(prop.Trim());
                if (v == null) return 0;
                var d = Convert.ToDouble(v, CultureInfo.InvariantCulture);
                return double.IsNaN(d) || double.IsInfinity(d) ? 0
                     : (d < 0 ? 0 : (d > 100 ? 100 : d));
            }
            catch { return 0; }
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

        private static int ExtractNumber(string json, string key)
        {
            var k = "\"" + key + "\"";
            var i = json.IndexOf(k, StringComparison.OrdinalIgnoreCase);
            if (i < 0) return -1;
            i = json.IndexOf(':', i + k.Length); if (i < 0) return -1;
            var j = i + 1;
            while (j < json.Length && (char.IsWhiteSpace(json[j]))) j++;
            var start = j;
            while (j < json.Length && char.IsDigit(json[j])) j++;
            return int.TryParse(json.Substring(start, j - start), out var n) ? n : -1;
        }
    }
}
