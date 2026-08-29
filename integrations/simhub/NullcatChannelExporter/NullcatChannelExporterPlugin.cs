// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
//
// nullCAT Channel Exporter - a deliberately dumb SimHub plugin.
//
// Sends one UDP line per data tick:
//   NULLCATX,<rpm>,<speedKmh>,<gear>,<clutchPct>,<throttlePct>
//
// That is the whole job. No shaping, no game-specific logic, no state:
// nullCAT owns all of that (the rig's ncxBindings config maps these
// channels onto its effects). Channel order here matches the example
// bindings in nullCAT's Docs/DEVICES.md; if you reorder or extend this
// line, update the bindings to match - the wire is just numbered slots.
//
// Configuration: NullcatChannelExporter.json next to this DLL,
//   { "host": "192.168.1.50", "port": 4444 }
// Defaults to 127.0.0.1:4444 when the file is absent or unreadable.

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
                }
            }
            catch { /* keep defaults */ }

            _target = new IPEndPoint(IPAddress.Parse(host), port);
            _udp = new UdpClient();
            SimHub.Logging.Current.Info(
                "NullcatChannelExporter: sending NULLCATX to " + _target);
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

            var line = string.Format(CultureInfo.InvariantCulture,
                "NULLCATX,{0:0.#},{1:0.##},{2:0},{3:0.#},{4:0.#}",
                d.Rpms, d.SpeedKmh, gear, d.Clutch, d.Throttle);

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
