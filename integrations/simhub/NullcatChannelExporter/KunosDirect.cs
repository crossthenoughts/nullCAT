// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
//
// The Assetto Corsa family's accelerations straight from the game's own
// shared memory, at the game's physics rate (about 333 samples a second
// against SimHub's 60), for nullCAT's belt and Impacts. Off unless the
// settings file says "kunosDirect": "on" or the SimHub action
// ToggleKunosDirect turns it on; the two paths can be compared by flipping
// it while driving.
//
// Read only: the physics page the game publishes for telemetry tools (the
// one SimHub itself reads), opened for reading; nothing is written and no
// process is touched.
//
// The page's values go out only once they match SimHub's own: for each of
// SimHub's accelerations (along, across, up), the page's g component that
// follows it best over the last half minute of driving, and the scale and
// offset that turn one into the other, so the sign, the unit and the zero
// stay SimHub's whatever the game's own convention. A value goes on its
// own short line, NULLCATY,accSurge=..,accSway=..,accHeave=.., on every
// new physics step, and only while the main line flows (a game SimHub
// reports paused or stopped goes quiet on both, so nullCAT lets its
// effects go); the main line leaves out what this one carries. Until a
// component matches clearly, whenever the match falls away, when the game
// pauses or stops, and with the switch off, SimHub's values go on the main
// line as before.
//
// With "kunosDirectHeights": "on" as well, the same line carries the height
// of the road under each tyre (the page's contact points) and, where the
// page has one (Competizione, EVO, Rally; plain Assetto Corsa's ends
// before it), the game's own kerb vibration, at the physics rate: a rumble
// strip's ribs pass under a tyre about 110 times a second at 100 km/h, too
// fast for SimHub's 60 a second to see.
//
// A new physics step is the packet counter moving or, for a game that does
// not move it, any value read changing.

using System;
using System.Diagnostics;
using System.Globalization;
using System.IO.MemoryMappedFiles;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;

namespace NullcatChannelExporter
{
    internal sealed class KunosDirect : IDisposable
    {
        // SPageFilePhysics (Assetto Corsa and Competizione): the packet
        // counter first, accG[3] (g) after gas, brake, fuel, gear, rpm,
        // steer angle, speed and velocity[3].
        private const string MapName = "Local\\acpmf_physics";
        private const int PacketIdOffset = 0, AccGOffset = 44, ViewBytes = 56;
        // TyreContactPoint[4] (x, y, z in m, y up) and kerbVibration, at
        // the offsets SimHub's own readers use for these pages; the view
        // covers them only with heights on (plain AC's page ends at 580).
        private const int ContactOffset = 420, ContactStride = 12, KerbOffset = 784;
        private const int ViewBytesAc = 580, ViewBytesExt = 800;

        // The match: an exponential window of about half a minute of
        // driving; decided after 10 s of it; a component counts when it
        // follows SimHub's value with a correlation of 0.85 or more and
        // stops counting below 0.7.
        private const double FitWindowSec = 30.0;
        private const double FitMinSec = 10.0;
        private const double FitMaxGapSec = 0.1;   // a longer gap between ticks counts as this much driving
        private const double MatchOn = 0.85, MatchOff = 0.7;
        private const double MovingKmh = 20.0;
        // No new physics step for this long: the game paused or stopped.
        private const long   StillMs = 100;
        // No main line for this long: SimHub has paused or stopped sending.
        private const long   MainQuietMs = 150;

        private static readonly string[] Tokens = { "accSurge", "accSway", "accHeave" };

        private readonly Action<string> _send;
        private readonly Action<string> _log;
        private readonly object _mapLock = new object();
        private MemoryMappedFile _mmf;
        private MemoryMappedViewAccessor _view;
        private DateTime _nextOpenTry = DateTime.MinValue;

        private volatile bool _enabled;
        private volatile bool _heights;    // the road heights (and kerb) ride the fast line too
        private readonly bool _kerbFast;   // the kerb may ride it (no curbs property bound in the settings file)
        private volatile bool _openTried;  // the page has been asked for at least once
        private double _lastSpeedKmh;      // the speed at the last Observe
        private volatile bool _ext;        // the running game's page reaches kerbVibration
        private volatile bool _openOk;     // the page is open
        private double _lastStepSec = -1;  // when Observe last saw a new physics step
        private double _lastSeenG0 = double.NaN, _lastSeenG1, _lastSeenG2;
        private string _lastLogged = "";
        private double _lastLogSec = -100;
        private volatile bool _gameOk;     // the running game is one of the family
        private volatile Map  _map;        // what the page's values are, as SimHub's
        private volatile bool _live;       // the fast line is carrying the matched values now
        private volatile bool _run = true;
        private int _sentThisSecond, _rate;
        private readonly Thread _thread;

        // Per SimHub axis (surge, sway, heave), per page component (0..2).
        private readonly Fit[,] _fit = new Fit[3, 3];
        private int _lastFitPacket = int.MinValue;
        private double _drivenSec, _nextDecideSec;
        private readonly Stopwatch _clock = Stopwatch.StartNew();
        private long _mainMs = long.MinValue / 2;   // when the main line last went
        private double _lastObserveSec = -1;

        private sealed class Map
        {
            public readonly int[] Src = { -1, -1, -1 };
            public readonly double[] K = new double[3], C = new double[3], R = new double[3];
            public bool Any { get { return Src[0] >= 0 || Src[1] >= 0 || Src[2] >= 0; } }
        }

        // Exponentially weighted means, variances and covariance of a pair.
        private sealed class Fit
        {
            public double N, Ms, Mh, Vs, Vh, Cv;
            // dt: the driving time this pair stands for.
            public void Add(double s, double h, double dt)
            {
                N++;
                if (N == 1) { Ms = s; Mh = h; Vs = Vh = Cv = 0; return; }
                var a = Math.Min(1.0, Math.Max(1.0 / N, dt / FitWindowSec));
                var ds = s - Ms; var dh = h - Mh;
                Ms += a * ds; Mh += a * dh;
                Vs = (1 - a) * (Vs + a * ds * ds);
                Vh = (1 - a) * (Vh + a * dh * dh);
                Cv = (1 - a) * (Cv + a * ds * dh);
            }
            public double R { get { return (Vs > 0 && Vh > 0) ? Cv / Math.Sqrt(Vs * Vh) : 0; } }
            // The ratio of the spreads, signed: the two are the same signal
            // read a moment apart, which a regression slope would scale down
            // by their correlation.
            public double K { get { return (Vs > 0 && Vh > 0) ? Math.Sign(Cv) * Math.Sqrt(Vs / Vh) : 0; } }
            public double C { get { return Ms - K * Mh; } }
        }

        // kerbFast: the game's kerb vibration may go on the fast line (false
        // when the settings file binds a curbs property: one source each).
        public KunosDirect(bool enabled, bool heights, bool kerbFast, Action<string> send, Action<string> log)
        {
            _enabled = enabled; _heights = heights; _kerbFast = kerbFast; _send = send; _log = log;
            ResetFit();
            _thread = new Thread(Loop) { IsBackground = true, Name = "nullCAT Kunos direct", Priority = ThreadPriority.AboveNormal };
            _thread.Start();
        }

        public bool Enabled
        {
            get { return _enabled; }
            set
            {
                if (_enabled == value) return;
                _enabled = value;
                if (!value) { _live = false; Close(); }
                _log("nullCAT Channel Exporter: Kunos direct accelerations " + (value ? "on" : "off"));
            }
        }

        // Whether the running game is one of the family (the direct read
        // exists for it).
        public bool GameOk { get { return _gameOk; } }

        // Called on SimHub's tick: whether the game is one of the family (a
        // different game starts the match over).
        // extended: the game's page reaches kerbVibration (all but plain AC).
        public void SetGame(bool acFamily, bool changed, bool extended)
        {
            // The game first, then the close: a reopen on the reader thread
            // sizes its view for this game, not the last one.
            _gameOk = acFamily;
            _ext = extended;
            if (changed) { _map = null; _live = false; ResetFit(); Close(); _lastStepSec = -1; }
        }

        // Whether the fast line carries this now (the main line then leaves
        // it out): 0 surge, 1 sway, 2 heave, 3 the road heights, 4 the kerb.
        public bool Carries(int axis)
        {
            if (!(_enabled && _gameOk && _live)) return false;
            if (axis == 3) return _heights;
            if (axis == 4) return _heights && _ext && _kerbFast;
            var m = _map;
            return m != null && m.Src[axis] >= 0;
        }

        // SimHub's values this tick against the page's, while moving.
        public void Observe(double? surge, double? sway, double? heave, double speedKmh)
        {
            var now = _clock.Elapsed.TotalSeconds;
            var dt = _lastObserveSec < 0 ? 0.0 : Math.Min(FitMaxGapSec, now - _lastObserveSec);
            _lastObserveSec = now;
            _lastSpeedKmh = speedKmh;
            if (!_enabled || !_gameOk || speedKmh < MovingKmh) return;
            int packet; double g0, g1, g2;
            if (!TryRead(out packet, out g0, out g1, out g2)) return;
            // A new step: the counter moved, or (a game that does not move
            // it) the values did.
            if (packet == _lastFitPacket && g0 == _lastSeenG0 && g1 == _lastSeenG1 && g2 == _lastSeenG2) return;
            _lastFitPacket = packet; _lastSeenG0 = g0; _lastSeenG1 = g1; _lastSeenG2 = g2;
            _lastStepSec = now;
            var sim = new[] { surge, sway, heave };
            var page = new[] { g0, g1, g2 };
            for (var a = 0; a < 3; a++)
            {
                if (!sim[a].HasValue || double.IsNaN(sim[a].Value) || double.IsInfinity(sim[a].Value)) continue;
                for (var j = 0; j < 3; j++) _fit[a, j].Add(sim[a].Value, page[j], dt);
            }
            _drivenSec += dt;
            if (_drivenSec >= FitMinSec && _drivenSec >= _nextDecideSec) { _nextDecideSec = _drivenSec + 1.0; Decide(); }
        }

        // What the status property shows.
        public string Status()
        {
            if (!_enabled) return "off";
            if (!_gameOk) return "on, waiting for an Assetto Corsa family game";
            var m = _map;
            var now = _clock.Elapsed.TotalSeconds;
            var driving = _lastSpeedKmh >= MovingKmh;
            // Below 20 km/h the page is not asked for (the match runs while
            // driving), so that is not the page's fault.
            if (!_openOk)
                return (_openTried && driving)
                    ? "on, the game's shared memory (" + MapName + ") is not open: is the game running?"
                    : "on, waiting to drive (the match starts above 20 km/h)";
            if (m == null || !m.Any)
            {
                if (driving && _lastObserveSec >= 0 && now - _lastObserveSec < 2 && (_lastStepSec < 0 || now - _lastStepSec > 2))
                    return "on, the game's page is not moving (no new physics step while driving): SimHub's values are sent" + (_heights ? HeightsNote() : "");
                return (_drivenSec < FitMinSec
                    ? "on, matching against SimHub (" + (int)(_drivenSec * 100 / FitMinSec) + "%, keep driving)"
                    : "on, no clear match: SimHub's values are sent") + (_heights ? HeightsNote() : "");
            }
            var sb = new StringBuilder(_live ? "on, live " + _rate + "/s:" : "on, matched (game paused or stopped):");
            for (var a = 0; a < 3; a++)
            {
                if (m.Src[a] < 0) { sb.Append(' ').Append(Tokens[a]).Append(" SimHub's;"); continue; }
                sb.Append(' ').Append(Tokens[a]).Append(" = ")
                  .Append(m.K[a].ToString("0.###", CultureInfo.InvariantCulture)).Append(" x accG[").Append(m.Src[a]).Append("] + ")
                  .Append(m.C[a].ToString("0.###", CultureInfo.InvariantCulture)).Append(" (r ")
                  .Append(m.R[a].ToString("0.###", CultureInfo.InvariantCulture)).Append(");");
            }
            return sb.ToString().TrimEnd(';') + (_heights ? HeightsNote() : "");
        }
        private string HeightsNote()
        {
            return _live ? "; road heights" + (_ext && _kerbFast ? " and kerb" : "") + " at the physics rate" : "; road heights waiting";
        }

        // Called each time the main line goes: the fast line runs only while
        // it does.
        public void MainLineSent()
        {
            Interlocked.Exchange(ref _mainMs, _clock.ElapsedMilliseconds);
        }
        private bool MainFlowing()
        {
            return _clock.ElapsedMilliseconds - Interlocked.Read(ref _mainMs) < MainQuietMs;
        }

        // Once a second, for the status rate.
        public void Tick()
        {
            _rate = Interlocked.Exchange(ref _sentThisSecond, 0);
            // SimHub's log says what the direct read is doing whenever that
            // changes (at most every 10 s), so a session's log shows why it
            // was or was not live without anyone watching the property.
            var now = _clock.Elapsed.TotalSeconds;
            if (!_enabled || now - _lastLogSec < 10) return;
            var s = Status();
            var key = s.Contains("live ") ? "live" : s;   // the rate moves every second
            if (key == _lastLogged) return;
            _lastLogged = key; _lastLogSec = now;
            _log("nullCAT Channel Exporter: Kunos direct " + s);
        }

        public void Dispose()
        {
            _enabled = false;   // nothing reopens the page from here on
            _run = false;
            try { _thread.Join(500); } catch { }
            Close();
        }

        private void ResetFit()
        {
            for (var a = 0; a < 3; a++) for (var j = 0; j < 3; j++) _fit[a, j] = new Fit();
            _drivenSec = 0; _nextDecideSec = 0; _lastFitPacket = int.MinValue;
        }

        // Each SimHub axis to the page component that follows it best; a
        // component claimed twice goes to the closer match only.
        private void Decide()
        {
            var old = _map;
            var m = new Map();
            for (var a = 0; a < 3; a++)
            {
                var best = -1; var bestR = 0.0;
                for (var j = 0; j < 3; j++)
                {
                    var r = Math.Abs(_fit[a, j].R);
                    if (_fit[a, j].N >= 100 && r > bestR) { bestR = r; best = j; }
                }
                var was = old != null && old.Src[a] == best && best >= 0;
                if (best < 0 || bestR < (was ? MatchOff : MatchOn)) continue;
                var f = _fit[a, best];
                if (double.IsNaN(f.K) || double.IsInfinity(f.K) || Math.Abs(f.K) < 1e-6) continue;
                m.Src[a] = best; m.K[a] = f.K; m.C[a] = f.C; m.R[a] = f.R;
            }
            for (var a = 0; a < 3; a++)
                for (var b = a + 1; b < 3; b++)
                    if (m.Src[a] >= 0 && m.Src[a] == m.Src[b])
                    {
                        if (Math.Abs(m.R[a]) >= Math.Abs(m.R[b])) m.Src[b] = -1; else m.Src[a] = -1;
                    }
            var changed = old == null;
            if (!changed) for (var a = 0; a < 3; a++) changed |= old.Src[a] != m.Src[a];
            _map = m;
            if (changed) _log("nullCAT Channel Exporter: Kunos direct " + Status());
        }

        private void Loop()
        {
            var tick = new FineTick();
            var lastPacket = int.MinValue;
            double l0 = double.NaN, l1 = 0, l2 = 0, lh = 0;
            var sinceStep = Stopwatch.StartNew();
            var sb = new StringBuilder(256);
            var hts = new double[4];
            while (_run)
            {
                var m = _map;
                var accel = m != null && m.Any;
                if (!_enabled || !_gameOk || (!accel && !_heights) || !MainFlowing())
                {
                    _live = false;
                    Thread.Sleep(20);
                    continue;
                }
                int packet; double g0, g1, g2, kerb; bool heightsOk, kerbOk;
                if (TryReadFull(out packet, out g0, out g1, out g2, hts, out heightsOk, out kerb, out kerbOk)
                    && (packet != lastPacket || g0 != l0 || g1 != l1 || g2 != l2 || (heightsOk && hts[0] + hts[1] + hts[2] + hts[3] != lh)))
                {
                    lastPacket = packet; l0 = g0; l1 = g1; l2 = g2; lh = heightsOk ? hts[0] + hts[1] + hts[2] + hts[3] : lh;
                    sinceStep.Restart();
                    var page = new[] { g0, g1, g2 };
                    sb.Clear().Append("NULLCATY");
                    if (accel)
                        for (var a = 0; a < 3; a++)
                        {
                            if (m.Src[a] < 0) continue;
                            var v = m.K[a] * page[m.Src[a]] + m.C[a];
                            if (double.IsNaN(v) || double.IsInfinity(v)) continue;
                            sb.Append(',').Append(Tokens[a]).Append('=').Append(v.ToString("0.####", CultureInfo.InvariantCulture));
                        }
                    if (_heights && heightsOk)
                        for (var w = 0; w < 4; w++)
                            sb.Append(",roadHeight").Append(WheelNames[w]).Append('=').Append((hts[w] * 1000.0).ToString("0.##", CultureInfo.InvariantCulture));
                    if (_heights && kerbOk && _kerbFast)
                        sb.Append(",curbs=").Append(Math.Max(0, Math.Min(100, kerb * 100.0)).ToString("0.#", CultureInfo.InvariantCulture));
                    if (sb.Length > 8)
                    {
                        _send(sb.ToString());
                        Interlocked.Increment(ref _sentThisSecond);
                        _live = true;
                    }
                }
                else if (sinceStep.ElapsedMilliseconds > StillMs) _live = false;
                tick.Wait();
            }
            tick.Dispose();
        }

        private static readonly string[] WheelNames = { "FL", "FR", "RL", "RR" };

        // The g, and with heights on the contact points' heights and the
        // kerb vibration, in one consistent read (the counter the same
        // before and after).
        private bool TryReadFull(out int packet, out double g0, out double g1, out double g2,
                                 double[] heights, out bool heightsOk, out double kerb, out bool kerbOk)
        {
            heightsOk = false; kerb = 0; kerbOk = false;
            if (!_heights) return TryRead(out packet, out g0, out g1, out g2);
            packet = 0; g0 = g1 = g2 = 0;
            lock (_mapLock)
            {
                if (_view == null && (!_enabled || !Open())) return false;
                try
                {
                    for (var attempt = 0; attempt < 3; attempt++)
                    {
                        var p1 = _view.ReadInt32(PacketIdOffset);
                        var a = _view.ReadSingle(AccGOffset);
                        var b = _view.ReadSingle(AccGOffset + 4);
                        var c = _view.ReadSingle(AccGOffset + 8);
                        var ok = true;
                        for (var w = 0; w < 4; w++)
                        {
                            var y = _view.ReadSingle(ContactOffset + w * ContactStride + 4);
                            if (float.IsNaN(y) || float.IsInfinity(y)) ok = false;
                            heights[w] = y;
                        }
                        float k = 0; var kOk = false;
                        if (_ext && _view.Capacity >= KerbOffset + 4)
                        {
                            k = _view.ReadSingle(KerbOffset);
                            kOk = !(float.IsNaN(k) || float.IsInfinity(k));
                        }
                        var p2 = _view.ReadInt32(PacketIdOffset);
                        if (p1 != p2) continue;
                        if (float.IsNaN(a) || float.IsNaN(b) || float.IsNaN(c) ||
                            float.IsInfinity(a) || float.IsInfinity(b) || float.IsInfinity(c)) return false;
                        packet = p1; g0 = a; g1 = b; g2 = c;
                        // All four at 0: a page that does not fill them.
                        heightsOk = ok && (heights[0] != 0 || heights[1] != 0 || heights[2] != 0 || heights[3] != 0);
                        kerb = k; kerbOk = kOk;
                        return true;
                    }
                    return false;
                }
                catch { CloseLocked(); return false; }
            }
        }

        // One consistent read of the page: the packet counter the same
        // before and after the three g.
        private bool TryRead(out int packet, out double g0, out double g1, out double g2)
        {
            packet = 0; g0 = g1 = g2 = 0;
            lock (_mapLock)
            {
                if (_view == null && (!_enabled || !Open())) return false;
                try
                {
                    for (var attempt = 0; attempt < 3; attempt++)
                    {
                        var p1 = _view.ReadInt32(PacketIdOffset);
                        var a = _view.ReadSingle(AccGOffset);
                        var b = _view.ReadSingle(AccGOffset + 4);
                        var c = _view.ReadSingle(AccGOffset + 8);
                        var p2 = _view.ReadInt32(PacketIdOffset);
                        if (p1 != p2) continue;
                        if (float.IsNaN(a) || float.IsNaN(b) || float.IsNaN(c) ||
                            float.IsInfinity(a) || float.IsInfinity(b) || float.IsInfinity(c)) return false;
                        packet = p1; g0 = a; g1 = b; g2 = c;
                        return true;
                    }
                    return false;
                }
                catch { CloseLocked(); return false; }
            }
        }

        private bool Open()
        {
            if (DateTime.UtcNow < _nextOpenTry) return false;
            _openTried = true;
            try
            {
                _mmf  = MemoryMappedFile.OpenExisting(MapName, MemoryMappedFileRights.Read);
                var bytes = !_heights ? ViewBytes : _ext ? ViewBytesExt : ViewBytesAc;
                try { _view = _mmf.CreateViewAccessor(0, bytes, MemoryMappedFileAccess.Read); }
                catch when (bytes > ViewBytesAc) { _view = _mmf.CreateViewAccessor(0, ViewBytesAc, MemoryMappedFileAccess.Read); }
                _openOk = true;
                return true;
            }
            catch
            {
                _openOk = false;
                CloseLocked();
                _nextOpenTry = DateTime.UtcNow.AddSeconds(1);
                return false;
            }
        }

        private void Close() { lock (_mapLock) CloseLocked(); }
        private void CloseLocked()
        {
            try { if (_view != null) _view.Dispose(); } catch { }
            try { if (_mmf != null) _mmf.Dispose(); } catch { }
            _view = null; _mmf = null;
        }

        // A 1 ms wait between looks at the page. Windows' sleep runs on a
        // 15.6 ms tick unless a program asks for a finer one, and Windows 11
        // ignores that ask from a program whose window is hidden (SimHub,
        // while you drive): a high-resolution waitable timer keeps 1 ms
        // regardless. Without one (Windows before 10 1803), Sleep(1).
        private sealed class FineTick : IDisposable
        {
            private IntPtr _timer;
            public FineTick()
            {
                try { _timer = CreateWaitableTimerExW(IntPtr.Zero, null, CreateWaitableTimerHighResolution, TimerAllAccess); }
                catch { _timer = IntPtr.Zero; }
            }
            public void Wait()
            {
                if (_timer != IntPtr.Zero)
                {
                    long due = -10000;   // 1 ms, relative, in 100 ns units
                    if (SetWaitableTimer(_timer, ref due, 0, IntPtr.Zero, IntPtr.Zero, false))
                    {
                        WaitForSingleObject(_timer, 50);
                        return;
                    }
                }
                Thread.Sleep(1);
            }
            public void Dispose()
            {
                if (_timer != IntPtr.Zero) { CloseHandle(_timer); _timer = IntPtr.Zero; }
            }
        }
        private const uint CreateWaitableTimerHighResolution = 0x00000002, TimerAllAccess = 0x1F0003;
        [DllImport("kernel32.dll", CharSet = CharSet.Unicode, SetLastError = true)]
        private static extern IntPtr CreateWaitableTimerExW(IntPtr attributes, string name, uint flags, uint access);
        [DllImport("kernel32.dll", SetLastError = true)]
        private static extern bool SetWaitableTimer(IntPtr timer, ref long dueTime, int period, IntPtr routine, IntPtr arg, bool resume);
        [DllImport("kernel32.dll", SetLastError = true)]
        private static extern uint WaitForSingleObject(IntPtr handle, uint ms);
        [DllImport("kernel32.dll", SetLastError = true)]
        private static extern bool CloseHandle(IntPtr handle);
    }
}
