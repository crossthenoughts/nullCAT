// SPDX-FileCopyrightText: 2026 Tim Palmgren (Ø Werks) <tim@zerowerks.co.nz>
// SPDX-License-Identifier: GPL-3.0-or-later
//
// The Assetto Corsa family straight from the game's own shared memory, at
// the game's physics rate (about 333 steps a second against SimHub's 60),
// for nullCAT's haptics and belt. Off unless the settings file says
// "kunosDirect": "on" or the SimHub action ToggleKunosDirect (or nullCAT's
// Direct telemetry switch) turns it on; the two paths can be compared by
// flipping it while driving.
//
// Read only: the physics page the game publishes for telemetry tools (the
// one SimHub itself reads), opened for reading; nothing is written and no
// process is touched.
//
// What goes: every channel the title's physics page has (KunosTitle below:
// where each value sits, as SimHub's own readers read the page, and how
// SimHub turns it into its own, which is kept so nothing on the rig
// changes), on a line of its own, NULLCATY,rpm=..,speedKmh=..,.., on every
// new physics step, and only while the main line flows (a game SimHub
// reports paused or stopped goes quiet on both, so nullCAT lets its effects
// go). The main line leaves out what this one carries. SimHub's values go
// for the rest: what the page lacks, what has not passed its check below, a
// channel the settings file binds to a property of its own, and everything
// with the switch off or the page not moving.
//
// The values whose sign and unit rest on a convention rather than on a
// field SimHub passes on as it is (the accelerations, the turn rates and,
// where the page also has contact patches, its slip angles) go only once
// they pass a check against the page's own physics over about half a
// minute of driving: the acceleration along the car against the change of
// its speed, across it against the speed times the yaw rate and the side
// the tyre loads move to, up against the change of the vertical speed,
// each turn rate against the change of the page's own heading, pitch and
// roll, the slip angles against the contact patches' own travel. A check
// decided holds for the game session: passed, the page's value goes;
// failed (its sign, unit or axis not what the table says), SimHub's goes
// and SimHub's log says why. Nothing is matched to SimHub's own values: in
// AC Rally they are wrong (SimHub works its accelerations out of the change
// of the car's local velocity, which leaves the turn out: its sway read
// about a third of the speed times the yaw rate).
//
// With "kunosDirectHeights": "on" as well, the same line carries the height
// of the road under each tyre (the page's contact points), for the Road
// tile and the belt's teleport check.
//
// A new physics step is the packet counter moving or, for a game that does
// not move it, any byte of the page changing.

using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.IO.MemoryMappedFiles;
using System.Runtime.InteropServices;
using System.Text;
using System.Threading;

namespace NullcatChannelExporter
{
    // What each title's physics page (Local\acpmf_physics) holds, and how
    // SimHub turns each value into its own. Offsets are SimHub's: its
    // readers (ACSharedMemory.dll) use one struct for Competizione, EVO and
    // Rally and a shorter one for plain Assetto Corsa, the same fields up to
    // 580 bytes. "Verified" below: read in those readers (the offset, and
    // how SimHub's own value is made from it). "Seen": in nullCAT
    // recordings of the game. "Assumed": neither; a value resting on an
    // assumption is checked against the page's own physics before it goes.
    internal sealed class KunosTitle
    {
        public string Key;          // SimHub's game name, letters and digits, lower case
        public string Name;
        public int PageBytes;       // how much of the page the title's struct covers
        public bool Loads;          // wheelLoad filled
        public bool Contacts;       // tyre contact points and headings filled
        public bool SlipRatio;      // slipRatio[4] (Competizione's struct on)
        public bool SlipAngle;      // slipAngle[4], radians
        public bool CheckSlipAngle; // ...checked against the contact patches (where the page has both)
        public bool Kerb;           // kerbVibration, 0..1
        public bool KerbNanIsZero;  // SimHub turns a NaN kerbVibration into 0 (no kerb)
        public bool Abs, Tc;        // SimHub's ABS / TC active is the page's abs / tc above 0
        public bool PitLimiter;     // SimHub's pit limiter is the page's pitLimiterOn
        public double RollDeg;      // SimHub's roll: this x the page's roll, in degrees
        public int RpmCap;          // SimHub caps the rpm here (0: no cap)
        // The page's local angular velocity (rad/s; x pitch, y yaw, z roll)
        // to the turn rates as SimHub gives them (deg/s: yaw the change of
        // the heading, a right turn positive; pitch minus the change of the
        // pitch; roll the change of SimHub's roll). Assumed, checked.
        public double YawSign, PitchSign, RollSign;

        // The page, in bytes (verified for every title: SimHub's struct
        // definitions, sequential, 4-byte fields).
        public const int PacketId = 0, Gas = 4, Brake = 8, Gear = 16, Rpms = 20, SpeedKmh = 28,
                         Velocity = 32, AccG = 44, WheelSlip = 56, WheelLoad = 72, WheelAngularSpeed = 104,
                         SuspensionTravel = 184, TcField = 204, Heading = 208, Pitch = 212, Roll = 216,
                         PitLimiterOn = 248, AbsField = 252, Turbo = 276, LocalAngularVelocity = 296,
                         Clutch = 364, TyreContactPoint = 420, TyreContactHeading = 516, LocalVelocity = 568,
                         SlipRatioField = 640, SlipAngleField = 656, KerbVibration = 784;
        public const int PageAc = 580, PageExt = 800;

        // Scales the wire wants, one place for the direct line and the
        // plugin's built-in bindings alike. The combined slip reads about 1
        // to 2 as a tyre starts to slide and 10 and up in a spin (SimHub's
        // own slip LEDs map 1..10): 6 = let go = 100 on the wire.
        public const double SlipScale = 100.0 / 6.0;
        public const double TravelScale = 1000.0;   // m -> mm
        public const double G = 9.80665;            // SimHub's accelerations: the page's g x this

        private static readonly KunosTitle[] All =
        {
            // Assetto Corsa: SimHub's ABS and TC active come from the game's
            // UDP feed and its pit limiter from the pit lane flag (verified),
            // so those three stay SimHub's (the page's abs and tc are the
            // car's settings). The page ends at 580: no slip ratio, slip
            // angle or kerb. Loads, contact points, wheel speeds, travel:
            // seen.
            new KunosTitle { Key = "assettocorsa", Name = "Assetto Corsa", PageBytes = PageAc,
                             Loads = true, Contacts = true, RollDeg = -1,
                             YawSign = -1, PitchSign = 1, RollSign = -1 },
            // Competizione: no loads (the game leaves wheelLoad at zero,
            // seen); ABS, TC and the pit limiter from the page as SimHub
            // reads them (verified).
            new KunosTitle { Key = "assettocorsacompetizione", Name = "Assetto Corsa Competizione", PageBytes = PageExt,
                             Contacts = true, SlipRatio = true, SlipAngle = true, CheckSlipAngle = true, Kerb = true,
                             Abs = true, Tc = true, PitLimiter = true, RollDeg = -1,
                             YawSign = -1, PitchSign = 1, RollSign = -1 },
            // EVO: no contact points (seen: all zero), so its own slip angles
            // go unchecked, as they did; SimHub reads the page's abs but gives
            // TC active as never (verified), so TC stays SimHub's. SimHub's
            // roll is the page's own sign here (verified).
            new KunosTitle { Key = "assettocorsaevo", Name = "Assetto Corsa EVO", PageBytes = PageExt,
                             Loads = true, SlipRatio = true, SlipAngle = true, Kerb = true,
                             Abs = true, PitLimiter = true, RollDeg = 1,
                             YawSign = -1, PitchSign = 1, RollSign = 1 },
            // Rally: loads, contact points, slip ratio and angle (seen);
            // SimHub caps the rpm at 10000 and turns a NaN kerb vibration
            // into 0 (verified). SimHub works its accelerations out of the
            // local velocity; the page's g go by the family's convention
            // (assumed, checked).
            new KunosTitle { Key = "assettocorsarally", Name = "Assetto Corsa Rally", PageBytes = PageExt,
                             Loads = true, Contacts = true, SlipRatio = true, SlipAngle = true, CheckSlipAngle = true,
                             Kerb = true, KerbNanIsZero = true,
                             Abs = true, Tc = true, PitLimiter = true, RollDeg = -1, RpmCap = 10000,
                             YawSign = -1, PitchSign = 1, RollSign = 1 },
        };

        // The title for SimHub's game name (letters and digits, lower case),
        // null outside the family.
        public static KunosTitle For(string key)
        {
            foreach (var t in All) if (t.Key == key) return t;
            return null;
        }
    }

    internal sealed class KunosDirect : IDisposable
    {
        // The channels the direct line can carry, one source each: the
        // main line asks Carries() for each before it sends its own.
        public enum Ch
        {
            Rpm, Speed, Gear, Pedals, Boost, Abs, Tc, PitLimiter, Limiter, Angles,
            Surge, Sway, Heave, YawRate, PitchRate, RollRate,
            WheelSpeed, Load, WheelSlip, SuspTravel, SlipRatio, SlipAngle, RoadHeight, Kerb,
            Count
        }

        private const string MapName = "Local\\acpmf_physics";
        // No new physics step for this long: the game paused or stopped, and
        // the main line takes every channel back.
        private const long StillMs = 100;
        // No main line for this long: SimHub has paused or stopped sending.
        private const long MainQuietMs = 150;
        private const double Deg = 180.0 / Math.PI;

        // The checks: blocks of about 50 ms of physics steps above 30 km/h,
        // in an exponential window of about half a minute of driving;
        // decided after 10 s of it with enough movement to judge by; given
        // up after two minutes of movement that never follows.
        private const double BlockSec = 0.05, BlockMaxSec = 0.2;
        private const double CheckKmh = 30.0;
        private const double FitWindowSec = 30.0, CheckMinSec = 10.0, CheckGiveUpSec = 120.0;
        private const double SwayLoadsMin = 0.4;   // the side the load moves to, for sway's sign

        private readonly Action<string> _send;
        private readonly Action<string> _log;
        private readonly bool[] _excluded = new bool[(int)Ch.Count];   // bound to a property of its own in the settings file
        private readonly object _mapLock = new object();
        private MemoryMappedFile _mmf;
        private MemoryMappedViewAccessor _view;
        private int _viewBytes;
        private DateTime _nextOpenTry = DateTime.MinValue;

        private volatile bool _enabled;
        private readonly bool _heights;    // the road heights ride the fast line too
        private volatile KunosTitle _title;   // the running game, null outside the family
        private volatile bool _openTried, _openOk;
        private volatile bool _live;       // the fast line is sending now
        private volatile bool _run = true;
        private volatile bool _resetPending = true;   // the reader starts the session's checks over
        private double _maxRpm;            // SimHub's, for the limiter (written by the main tick)
        private readonly long[] _sentMs = new long[(int)Ch.Count];
        private int _sentThisSecond, _rate;
        private string _lastLogged = "";
        private double _lastLogSec = -100;
        private readonly Stopwatch _clock = Stopwatch.StartNew();
        private long _mainMs = long.MinValue / 2;   // when the main line last went
        private readonly Thread _thread;
        private volatile Check[] _checks = new Check[0];

        // excluded: channels the settings file binds to properties of its
        // own (the file's win: one source each).
        public KunosDirect(bool enabled, bool heights, IEnumerable<Ch> excluded, Action<string> send, Action<string> log)
        {
            _enabled = enabled; _heights = heights; _send = send; _log = log;
            if (excluded != null) foreach (var c in excluded) _excluded[(int)c] = true;
            ForgetSent();
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
                if (!value) { _live = false; ForgetSent(); Close(); }
                _log("nullCAT Channel Exporter: Kunos direct " + (value ? "on" : "off"));
            }
        }

        // Whether the running game is one of the family (the direct read
        // exists for it).
        public bool GameOk { get { return _title != null; } }

        // Called on SimHub's tick with the running game (null outside the
        // family); a different game starts its checks over.
        public void SetGame(KunosTitle title, bool changed)
        {
            if (!changed) return;
            // The game first, then the close: a reopen on the reader thread
            // sizes its view for this game, not the last one.
            _title = title;
            _live = false;
            ForgetSent();
            _resetPending = true;
            Close();
        }

        // SimHub's max rpm, each tick, for the limiter the fast line works
        // out from the page's rpm as the main line does.
        public void SetMaxRpm(double maxRpm)
        {
            Interlocked.Exchange(ref _maxRpm, maxRpm);
        }

        // Whether the fast line carries this channel now (the main line then
        // leaves it out): it went within the last 100 ms.
        public bool Carries(Ch c)
        {
            if (!_enabled || _title == null) return false;
            return _clock.ElapsedMilliseconds - Interlocked.Read(ref _sentMs[(int)c]) < StillMs;
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

        private void ForgetSent()
        {
            for (var i = 0; i < _sentMs.Length; i++) Interlocked.Exchange(ref _sentMs[i], long.MinValue / 2);
        }

        // What the status property shows.
        public string Status()
        {
            if (!_enabled) return "off";
            var t = _title;
            if (t == null) return "on, waiting for an Assetto Corsa family game";
            if (!_openOk)
                return _openTried ? "on, " + t.Name + ": the game's shared memory (" + MapName + ") is not open: is the game running?"
                                  : "on, " + t.Name + ": waiting for SimHub's data";
            if (!_live)
                return "on, " + t.Name + ": the game's page is not moving (paused, or no new physics step): SimHub's values are sent";
            var sb = new StringBuilder("on, live " + _rate + "/s, " + t.Name + ", from the page:");
            var any = false;
            foreach (var c in StatusChannels)
            {
                if (c == Ch.SlipAngle && t.CheckSlipAngle) continue;   // listed with the checks
                if (Carries(c)) { sb.Append(any ? ", " : " ").Append(ChName(c)); any = true; }
            }
            if (!any) sb.Append(" none");
            var checks = _checks;
            if (checks.Length > 0)
            {
                sb.Append("; checked:");
                for (var i = 0; i < checks.Length; i++)
                    sb.Append(i > 0 ? ", " : " ").Append(checks[i].Token).Append(' ').Append(checks[i].Short());
            }
            return sb.ToString();
        }
        private static readonly Ch[] StatusChannels =
        {
            Ch.Rpm, Ch.Speed, Ch.Gear, Ch.Pedals, Ch.Boost, Ch.Abs, Ch.Tc, Ch.PitLimiter, Ch.Limiter, Ch.Angles,
            Ch.WheelSpeed, Ch.Load, Ch.WheelSlip, Ch.SuspTravel, Ch.SlipRatio, Ch.SlipAngle, Ch.RoadHeight, Ch.Kerb
        };
        private static string ChName(Ch c)
        {
            switch (c)
            {
                case Ch.Rpm: return "rpm";
                case Ch.Speed: return "speed";
                case Ch.Gear: return "gear";
                case Ch.Pedals: return "pedals";
                case Ch.Boost: return "boost";
                case Ch.Abs: return "ABS";
                case Ch.Tc: return "TC";
                case Ch.PitLimiter: return "pit limiter";
                case Ch.Limiter: return "limiter";
                case Ch.Angles: return "pitch and roll";
                case Ch.WheelSpeed: return "wheel speeds";
                case Ch.Load: return "loads";
                case Ch.WheelSlip: return "combined slip";
                case Ch.SuspTravel: return "suspension travel";
                case Ch.SlipRatio: return "slip ratios";
                case Ch.SlipAngle: return "slip angles";
                case Ch.RoadHeight: return "road heights";
                case Ch.Kerb: return "kerb vibration";
                default: return c.ToString();
            }
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
            var key = s.StartsWith("on, live ", StringComparison.Ordinal) ? StripRate(s) : s;   // the rate moves every second
            if (key == _lastLogged) return;
            _lastLogged = key; _lastLogSec = now;
            _log("nullCAT Channel Exporter: Kunos direct " + s);
        }
        private static string StripRate(string s)
        {
            var i = s.IndexOf("/s, ", StringComparison.Ordinal);
            return i < 0 ? s : "on, live" + s.Substring(i + 2);
        }

        public void Dispose()
        {
            _enabled = false;   // nothing reopens the page from here on
            _run = false;
            try { _thread.Join(500); } catch { }
            Close();
        }

        // ---- the page --------------------------------------------------------

        // One physics step as the page has it.
        private sealed class Sample
        {
            public int Gear, Rpm, PitLim;
            public double Gas, Brake, Clutch, SpeedKmh, Turbo, Heading, Pitch, Roll, Abs, Tc, Kerb;
            public readonly double[] Vel = new double[3], AccG = new double[3], AngVel = new double[3], LocVel = new double[3];
            public readonly double[] WheelSlip = new double[4], Load = new double[4], WheelSpeed = new double[4], Travel = new double[4];
            public readonly double[] CpX = new double[4], CpY = new double[4], CpZ = new double[4], HdX = new double[4], HdZ = new double[4];
            public readonly double[] SlipRatio = new double[4], SlipAngle = new double[4];
            public bool Ext;   // the view reaches Competizione's fields
        }

        private static double F(byte[] b, int off) { return BitConverter.ToSingle(b, off); }
        private static void Fn(byte[] b, int off, double[] into) { for (var i = 0; i < into.Length; i++) into[i] = BitConverter.ToSingle(b, off + 4 * i); }

        private static void Parse(byte[] b, int n, Sample s)
        {
            s.Gas = F(b, KunosTitle.Gas); s.Brake = F(b, KunosTitle.Brake);
            s.Gear = BitConverter.ToInt32(b, KunosTitle.Gear); s.Rpm = BitConverter.ToInt32(b, KunosTitle.Rpms);
            s.SpeedKmh = F(b, KunosTitle.SpeedKmh);
            Fn(b, KunosTitle.Velocity, s.Vel); Fn(b, KunosTitle.AccG, s.AccG);
            Fn(b, KunosTitle.WheelSlip, s.WheelSlip); Fn(b, KunosTitle.WheelLoad, s.Load);
            Fn(b, KunosTitle.WheelAngularSpeed, s.WheelSpeed); Fn(b, KunosTitle.SuspensionTravel, s.Travel);
            s.Tc = F(b, KunosTitle.TcField);
            s.Heading = F(b, KunosTitle.Heading); s.Pitch = F(b, KunosTitle.Pitch); s.Roll = F(b, KunosTitle.Roll);
            s.PitLim = BitConverter.ToInt32(b, KunosTitle.PitLimiterOn); s.Abs = F(b, KunosTitle.AbsField);
            s.Turbo = F(b, KunosTitle.Turbo); Fn(b, KunosTitle.LocalAngularVelocity, s.AngVel);
            s.Clutch = F(b, KunosTitle.Clutch);
            for (var w = 0; w < 4; w++)
            {
                var p = KunosTitle.TyreContactPoint + 12 * w;
                s.CpX[w] = F(b, p); s.CpY[w] = F(b, p + 4); s.CpZ[w] = F(b, p + 8);
                var h = KunosTitle.TyreContactHeading + 12 * w;
                s.HdX[w] = F(b, h); s.HdZ[w] = F(b, h + 8);
            }
            Fn(b, KunosTitle.LocalVelocity, s.LocVel);
            s.Ext = n >= KunosTitle.PageExt;
            if (s.Ext)
            {
                Fn(b, KunosTitle.SlipRatioField, s.SlipRatio); Fn(b, KunosTitle.SlipAngleField, s.SlipAngle);
                s.Kerb = F(b, KunosTitle.KerbVibration);
            }
        }

        // One consistent copy of the page: the packet counter the same before
        // and after. False when the page is not open or cannot be read.
        private bool ReadPage(KunosTitle t, byte[] buf, out int n)
        {
            n = 0;
            lock (_mapLock)
            {
                if (_view == null && (!_enabled || !Open(t))) return false;
                try
                {
                    n = Math.Min(_viewBytes, buf.Length);
                    for (var attempt = 0; attempt < 3; attempt++)
                    {
                        var p1 = _view.ReadInt32(KunosTitle.PacketId);
                        _view.ReadArray(0, buf, 0, n);
                        if (_view.ReadInt32(KunosTitle.PacketId) == p1) return true;
                    }
                    return false;
                }
                catch { CloseLocked(); return false; }
            }
        }

        private bool Open(KunosTitle t)
        {
            if (DateTime.UtcNow < _nextOpenTry) return false;
            _openTried = true;
            try
            {
                _mmf = MemoryMappedFile.OpenExisting(MapName, MemoryMappedFileRights.Read);
                var bytes = t.PageBytes;
                try { _view = _mmf.CreateViewAccessor(0, bytes, MemoryMappedFileAccess.Read); }
                catch when (bytes > KunosTitle.PageAc)
                {
                    bytes = KunosTitle.PageAc;
                    _view = _mmf.CreateViewAccessor(0, bytes, MemoryMappedFileAccess.Read);
                }
                _viewBytes = bytes;
                _openOk = true;
                _log("nullCAT Channel Exporter: Kunos direct, " + t.Name + ": the game's page is open (" + bytes + " bytes read)");
                return true;
            }
            catch
            {
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
            _openOk = false;
        }

        // ---- the fast line ---------------------------------------------------

        private void Loop()
        {
            var tick = new FineTick();
            var buf = new byte[KunosTitle.PageExt];
            var last = new byte[KunosTitle.PageExt];
            var lastN = 0;
            var s = new Sample();
            var sb = new StringBuilder(1024);
            var sinceStep = Stopwatch.StartNew();
            var block = new Block();
            KunosTitle checksFor = null;
            while (_run)
            {
                var t = _title;
                if (!_enabled || t == null || !MainFlowing())
                {
                    _live = false;
                    Thread.Sleep(20);
                    continue;
                }
                if (_resetPending || checksFor != t)
                {
                    _resetPending = false;
                    checksFor = t;
                    _checks = NewChecks(t);
                    block.Reset();
                    lastN = 0;
                }
                int n;
                if (!ReadPage(t, buf, out n))
                {
                    _live = false;
                    Thread.Sleep(20);   // not open: Open retries once a second
                    continue;
                }
                if (n != lastN || !Same(buf, last, n))
                {
                    Buffer.BlockCopy(buf, 0, last, 0, n);
                    lastN = n;
                    sinceStep.Restart();
                    Parse(buf, n, s);
                    AddToChecks(t, s, _clock.Elapsed.TotalSeconds, block);
                    sb.Clear().Append("NULLCATY");
                    Build(t, s, sb, _clock.ElapsedMilliseconds);
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

        private static bool Same(byte[] a, byte[] b, int n)
        {
            for (var i = 0; i < n; i++) if (a[i] != b[i]) return false;
            return true;
        }

        // Whether the fast line may carry a channel for this title now.
        private bool Allowed(KunosTitle t, Sample s, Ch c)
        {
            if (_excluded[(int)c]) return false;
            switch (c)
            {
                case Ch.Abs: return t.Abs;
                case Ch.Tc: return t.Tc;
                case Ch.PitLimiter: return t.PitLimiter;
                case Ch.Load: return t.Loads;
                case Ch.SlipRatio: return t.SlipRatio && s.Ext;
                case Ch.SlipAngle: return t.SlipAngle && s.Ext && (!t.CheckSlipAngle || Passed(Ch.SlipAngle));
                case Ch.Kerb: return t.Kerb && s.Ext;
                case Ch.RoadHeight: return t.Contacts && _heights;
                case Ch.Surge: case Ch.Sway: case Ch.Heave: case Ch.YawRate: case Ch.PitchRate: case Ch.RollRate:
                    return Passed(c);
                default: return true;
            }
        }

        private bool Passed(Ch c)
        {
            foreach (var k in _checks) if (k.Chan == c) return k.State > 0;
            return false;
        }

        private void Mark(Ch c, long nowMs) { Interlocked.Exchange(ref _sentMs[(int)c], nowMs); }

        private static bool Finite(double v) { return !(double.IsNaN(v) || double.IsInfinity(v)); }

        private static void Put(StringBuilder sb, string token, double v, string fmt)
        {
            sb.Append(',').Append(token).Append('=').Append(v.ToString(fmt, CultureInfo.InvariantCulture));
        }

        // A per-wheel group, all four or none (all four at exactly 0, with
        // zeroIsEmpty: a page that does not fill it).
        private static bool PutWheels(StringBuilder sb, string token, double[] v, double scale, string fmt, bool zeroIsEmpty)
        {
            for (var w = 0; w < 4; w++) if (!Finite(v[w])) return false;
            if (zeroIsEmpty && v[0] == 0 && v[1] == 0 && v[2] == 0 && v[3] == 0) return false;
            for (var w = 0; w < 4; w++) Put(sb, token + Wheels[w], v[w] * scale, fmt);
            return true;
        }
        private static readonly string[] Wheels = { "FL", "FR", "RL", "RR" };

        // The line for one physics step: each channel the title has, in the
        // unit and sign SimHub gives it (see KunosTitle), marked as carried.
        private void Build(KunosTitle t, Sample s, StringBuilder sb, long nowMs)
        {
            // The engine, the pedals, the flags: SimHub's own formulas
            // (verified, every title).
            var rpm = t.RpmCap > 0 ? Math.Min(s.Rpm, t.RpmCap) : s.Rpm;
            if (Allowed(t, s, Ch.Rpm)) { Put(sb, "rpm", rpm, "0.#"); Mark(Ch.Rpm, nowMs); }
            if (Allowed(t, s, Ch.Speed) && Finite(s.SpeedKmh)) { Put(sb, "speedKmh", s.SpeedKmh, "0.##"); Mark(Ch.Speed, nowMs); }
            // The page's gear: 0 reverse, 1 neutral, 2 first; the wire's -1, 0, 1.
            if (Allowed(t, s, Ch.Gear)) { Put(sb, "gear", s.Gear - 1, "0"); Mark(Ch.Gear, nowMs); }
            if (Allowed(t, s, Ch.Pedals) && Finite(s.Gas) && Finite(s.Brake) && Finite(s.Clutch))
            {
                Put(sb, "throttlePct", s.Gas * 100.0, "0.#");
                Put(sb, "brakePct", s.Brake * 100.0, "0.#");
                Put(sb, "clutchPct", 100.0 - s.Clutch * 100.0, "0.#");   // the page's clutch is 1 with the pedal up
                Mark(Ch.Pedals, nowMs);
            }
            if (Allowed(t, s, Ch.Boost) && Finite(s.Turbo)) { Put(sb, "boost", s.Turbo, "0.###"); Mark(Ch.Boost, nowMs); }
            if (Allowed(t, s, Ch.Abs)) { Put(sb, "absActive", s.Abs > 0 ? 1 : 0, "0"); Mark(Ch.Abs, nowMs); }
            if (Allowed(t, s, Ch.Tc)) { Put(sb, "tcActive", s.Tc > 0 ? 1 : 0, "0"); Mark(Ch.Tc, nowMs); }
            if (Allowed(t, s, Ch.PitLimiter)) { Put(sb, "pitLimiter", s.PitLim > 0 ? 1 : 0, "0"); Mark(Ch.PitLimiter, nowMs); }
            // The limiter as the main line works it out (rpm within 1.5 % of
            // SimHub's max rpm), from the page's rpm.
            if (Allowed(t, s, Ch.Rpm) && Allowed(t, s, Ch.Limiter))
            {
                var max = Interlocked.CompareExchange(ref _maxRpm, 0.0, 0.0);
                Put(sb, "limiter", max > 0 && rpm >= max * 0.985 ? 1 : 0, "0");
                Mark(Ch.Limiter, nowMs);
            }
            // Pitch and roll, degrees, SimHub's signs (verified).
            if (Allowed(t, s, Ch.Angles) && Finite(s.Pitch) && Finite(s.Roll))
            {
                Put(sb, "pitchDeg", s.Pitch * Deg, "0.####");
                Put(sb, "rollDeg", t.RollDeg * s.Roll * Deg, "0.####");
                Mark(Ch.Angles, nowMs);
            }

            // The body's motion, once checked, in SimHub's signs: surge minus
            // the page's g along (braking positive), sway its g across (a
            // left turn positive), heave its g up, m/s^2 (verified in
            // SimHub's readers for AC, Competizione and EVO, and seen in
            // recordings; the same convention assumed for Rally). The turn
            // rates as KunosTitle says.
            if (Allowed(t, s, Ch.Surge) && Finite(s.AccG[2])) { Put(sb, "accSurge", -s.AccG[2] * KunosTitle.G, "0.####"); Mark(Ch.Surge, nowMs); }
            if (Allowed(t, s, Ch.Sway) && Finite(s.AccG[0])) { Put(sb, "accSway", s.AccG[0] * KunosTitle.G, "0.####"); Mark(Ch.Sway, nowMs); }
            if (Allowed(t, s, Ch.Heave) && Finite(s.AccG[1])) { Put(sb, "accHeave", s.AccG[1] * KunosTitle.G, "0.####"); Mark(Ch.Heave, nowMs); }
            if (Allowed(t, s, Ch.YawRate) && Finite(s.AngVel[1])) { Put(sb, "yawRate", t.YawSign * s.AngVel[1] * Deg, "0.####"); Mark(Ch.YawRate, nowMs); }
            if (Allowed(t, s, Ch.PitchRate) && Finite(s.AngVel[0])) { Put(sb, "pitchRate", t.PitchSign * s.AngVel[0] * Deg, "0.####"); Mark(Ch.PitchRate, nowMs); }
            if (Allowed(t, s, Ch.RollRate) && Finite(s.AngVel[2])) { Put(sb, "rollRate", t.RollSign * s.AngVel[2] * Deg, "0.####"); Mark(Ch.RollRate, nowMs); }

            // Per wheel, at the scales of the plugin's built-in bindings.
            if (Allowed(t, s, Ch.WheelSpeed) && PutWheels(sb, "wheelSpeed", s.WheelSpeed, 1.0, "0.####", false)) Mark(Ch.WheelSpeed, nowMs);
            if (Allowed(t, s, Ch.Load) && PutWheels(sb, "load", s.Load, 1.0, "0.####", true)) Mark(Ch.Load, nowMs);
            if (Allowed(t, s, Ch.WheelSlip) && PutWheels(sb, "wheelSlip", s.WheelSlip, KunosTitle.SlipScale, "0.####", false)) Mark(Ch.WheelSlip, nowMs);
            if (Allowed(t, s, Ch.SuspTravel) && PutWheels(sb, "suspTravel", s.Travel, KunosTitle.TravelScale, "0.####", false)) Mark(Ch.SuspTravel, nowMs);
            if (Allowed(t, s, Ch.SlipRatio) && PutWheels(sb, "slipRatio", s.SlipRatio, 1.0, "0.####", false)) Mark(Ch.SlipRatio, nowMs);
            // Slip angles from 5 km/h, as the main line sends them.
            if (Allowed(t, s, Ch.SlipAngle) && s.SpeedKmh > 5.0 && PutWheels(sb, "slipAngle", s.SlipAngle, Deg, "0.###", false)) Mark(Ch.SlipAngle, nowMs);
            // The road under each tyre, mm (hills and all: nullCAT filters).
            if (Allowed(t, s, Ch.RoadHeight) && PutWheels(sb, "roadHeight", s.CpY, 1000.0, "0.##", true)) Mark(Ch.RoadHeight, nowMs);
            // The game's own kerb vibration, 0..100: what nullCAT's Rumble
            // strip plays in the titles that have it.
            var kerb = !Finite(s.Kerb) && t.KerbNanIsZero ? 0.0 : s.Kerb;
            if (Allowed(t, s, Ch.Kerb) && Finite(kerb))
            {
                Put(sb, "curbs", Math.Max(0, Math.Min(100, kerb * 100.0)), "0.#");
                Mark(Ch.Kerb, nowMs);
            }
        }

        // ---- the checks ------------------------------------------------------

        // Exponentially weighted means, variances and covariance of a pair:
        // the page's value (s) and what the page's own physics says it
        // should be (h).
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
            // The ratio of the spreads, signed: the two are the same quantity
            // worked out two ways, which a regression slope would scale down
            // by their correlation.
            public double K { get { return (Vs > 0 && Vh > 0) ? Math.Sign(Cv) * Math.Sqrt(Vs / Vh) : 0; } }
        }

        private sealed class Check
        {
            public Ch Chan; public string Token, Against;
            public double RMin, KLo, KHi, Floor;   // the correlation it needs, the range of the ratio, the movement to judge by
            public bool SizeOnly;                  // judged on the size alone, its sign from Loads (sway)
            public readonly Fit Fit = new Fit();
            public Fit Loads;                      // sway: against the side the load moves to
            public string LoadsWhat;
            public double Driven, NextDecide = CheckMinSec;
            public volatile int State;             // 0 checking, 1 passed, -1 failed
            public volatile int Pct;               // of the first 10 s of driving
            public volatile bool Still;            // too little movement yet to judge by
            public volatile string Note = "";

            public string Short()
            {
                if (State > 0) return "ok";
                if (State < 0) return "SimHub's (" + Note + ")";
                if (Pct < 100) return "checking (" + Pct + " %)";
                return Still ? "checking (waiting for more movement)" : "checking (not clear yet)";
            }
        }

        private static Check NewCheck(Ch c, string token, string against, double rMin, double kLo, double kHi, double floor)
        {
            return new Check { Chan = c, Token = token, Against = against, RMin = rMin, KLo = kLo, KHi = kHi, Floor = floor };
        }

        private static Check[] NewChecks(KunosTitle t)
        {
            var sway = NewCheck(Ch.Sway, "accSway", "the speed times the yaw rate", 0.7, 0.6, 1.6, 1.0);
            sway.SizeOnly = true;
            sway.Loads = new Fit();
            sway.LoadsWhat = t.Loads ? "the tyre loads" : "the suspension travel";
            var list = new List<Check>
            {
                NewCheck(Ch.Surge, "accSurge", "the change of speed", 0.7, 0.6, 1.6, 0.5),
                sway,
                NewCheck(Ch.Heave, "accHeave", "the change of the vertical speed", 0.6, 0.6, 1.6, 0.3),
                NewCheck(Ch.YawRate, "yawRate", "the change of the heading", 0.7, 0.7, 1.4, 3.0),
                NewCheck(Ch.PitchRate, "pitchRate", "the change of the pitch", 0.5, 0.6, 1.6, 0.5),
                NewCheck(Ch.RollRate, "rollRate", "the change of the roll", 0.5, 0.6, 1.6, 0.5),
            };
            if (t.CheckSlipAngle) list.Add(NewCheck(Ch.SlipAngle, "slipAngle", "the contact patches' travel", 0.6, 0.6, 1.6, 0.5));
            return list.ToArray();
        }

        // About 50 ms of physics steps: the first and last of each value the
        // checks take the change of, and the sums of those they average.
        private sealed class Block
        {
            public int N;
            public double T0, T1, Speed0, Speed1, Vy0, Vy1, Head0, Head1, Pitch0, Pitch1, Roll0, Roll1;
            public double SAx, SAy, SAz, SWx, SWy, SWz, SVw, SLt;
            public bool CpOk;
            public readonly double[] Cx0 = new double[4], Cz0 = new double[4], Hx0 = new double[4], Hz0 = new double[4],
                                     Cx1 = new double[4], Cz1 = new double[4], Hx1 = new double[4], Hz1 = new double[4],
                                     SSlip = new double[4];
            public void Reset() { N = 0; }
        }

        // The side the load moves to, right minus left: the outside of a
        // turn (the right in a left turn) whatever the game's axes. The
        // tyre loads where the page has them, else the suspension travel
        // (more compressed on the outside).
        private static double LoadSide(KunosTitle t, Sample s)
        {
            if (t.Loads)
            {
                var tot = s.Load[0] + s.Load[1] + s.Load[2] + s.Load[3];
                return tot > 0 ? ((s.Load[1] + s.Load[3]) - (s.Load[0] + s.Load[2])) / tot : double.NaN;
            }
            return (s.Travel[1] + s.Travel[3]) - (s.Travel[0] + s.Travel[2]);
        }

        private static double Unwrap(double d)
        {
            while (d > Math.PI) d -= 2 * Math.PI;
            while (d < -Math.PI) d += 2 * Math.PI;
            return d;
        }

        private void AddToChecks(KunosTitle t, Sample s, double now, Block b)
        {
            var checks = _checks;
            var open = false;
            foreach (var k in checks) if (k.State == 0) { open = true; break; }
            if (!open) return;
            var lt = LoadSide(t, s);
            if (!(Finite(s.SpeedKmh) && Finite(s.Vel[1]) && Finite(s.Heading) && Finite(s.Pitch) && Finite(s.Roll) && Finite(lt)
                  && Finite(s.AccG[0]) && Finite(s.AccG[1]) && Finite(s.AccG[2]) && Finite(s.LocVel[2])
                  && Finite(s.AngVel[0]) && Finite(s.AngVel[1]) && Finite(s.AngVel[2])))
            { b.Reset(); return; }
            if (b.N > 0 && now - b.T1 > BlockMaxSec) b.Reset();   // a gap: start afresh
            var cpOk = t.CheckSlipAngle && s.Ext;
            for (var w = 0; w < 4 && cpOk; w++)
                cpOk = Finite(s.CpX[w]) && Finite(s.CpZ[w]) && Finite(s.HdX[w]) && Finite(s.HdZ[w]) && Finite(s.SlipAngle[w]);
            if (b.N == 0) Begin(b, s, now, cpOk);
            Accumulate(b, s, now, lt, cpOk);
            if (b.N >= 3 && b.T1 - b.T0 >= BlockSec)
            {
                Finish(t, b, checks);
                // This step ends one block and starts the next.
                Begin(b, s, now, cpOk);
                Accumulate(b, s, now, lt, cpOk);
            }
        }

        private static void Begin(Block b, Sample s, double now, bool cpOk)
        {
            b.N = 0; b.T0 = now;
            b.Speed0 = s.SpeedKmh / 3.6; b.Vy0 = s.Vel[1]; b.Head0 = s.Heading; b.Pitch0 = s.Pitch; b.Roll0 = s.Roll;
            b.SAx = b.SAy = b.SAz = b.SWx = b.SWy = b.SWz = b.SVw = b.SLt = 0;
            b.CpOk = cpOk;
            for (var w = 0; w < 4; w++) { b.Cx0[w] = s.CpX[w]; b.Cz0[w] = s.CpZ[w]; b.Hx0[w] = s.HdX[w]; b.Hz0[w] = s.HdZ[w]; b.SSlip[w] = 0; }
        }

        private static void Accumulate(Block b, Sample s, double now, double lt, bool cpOk)
        {
            b.N++; b.T1 = now;
            b.Speed1 = s.SpeedKmh / 3.6; b.Vy1 = s.Vel[1]; b.Head1 = s.Heading; b.Pitch1 = s.Pitch; b.Roll1 = s.Roll;
            b.SAx += s.AccG[0]; b.SAy += s.AccG[1]; b.SAz += s.AccG[2];
            b.SWx += s.AngVel[0]; b.SWy += s.AngVel[1]; b.SWz += s.AngVel[2];
            b.SVw += s.LocVel[2] * s.AngVel[1];
            b.SLt += lt;
            b.CpOk &= cpOk;
            for (var w = 0; w < 4; w++)
            {
                b.Cx1[w] = s.CpX[w]; b.Cz1[w] = s.CpZ[w]; b.Hx1[w] = s.HdX[w]; b.Hz1[w] = s.HdZ[w];
                b.SSlip[w] += Math.Abs(s.SlipAngle[w]);
            }
        }

        // A block's values against what the page's own physics says they
        // should be, into each open check.
        private void Finish(KunosTitle t, Block b, Check[] checks)
        {
            var dt = b.T1 - b.T0;
            if (dt <= 0 || dt > BlockMaxSec) return;
            var kmh = CheckKmh / 3.6;
            if (b.Speed0 < kmh || b.Speed1 < kmh) return;   // driving only
            var n = (double)b.N;
            var dv = (b.Speed1 - b.Speed0) / dt;
            var dvy = (b.Vy1 - b.Vy0) / dt;
            var dHead = Unwrap(b.Head1 - b.Head0) / dt * Deg;
            var dPitch = Unwrap(b.Pitch1 - b.Pitch0) / dt * Deg;
            var dRoll = Unwrap(b.Roll1 - b.Roll0) / dt * Deg;
            // A reset, a jump to the pits, a crash's spike: nothing a car does
            // in 50 ms of driving.
            if (Math.Abs(dv) > 50 || Math.Abs(dvy) > 50 || Math.Abs(dHead) > 200 || Math.Abs(dPitch) > 200 || Math.Abs(dRoll) > 200) return;
            var g = KunosTitle.G;
            foreach (var k in checks)
            {
                if (k.State != 0) continue;
                switch (k.Chan)
                {
                    case Ch.Surge: k.Fit.Add(-b.SAz / n * g, -dv, dt); break;
                    case Ch.Sway:
                        k.Fit.Add(b.SAx / n * g, b.SVw / n, dt);
                        k.Loads.Add(b.SAx / n * g, b.SLt / n, dt);
                        break;
                    case Ch.Heave: k.Fit.Add(b.SAy / n * g, dvy, dt); break;
                    case Ch.YawRate: k.Fit.Add(t.YawSign * b.SWy / n * Deg, dHead, dt); break;
                    case Ch.PitchRate: k.Fit.Add(t.PitchSign * b.SWx / n * Deg, -dPitch, dt); break;
                    case Ch.RollRate: k.Fit.Add(t.RollSign * b.SWz / n * Deg, t.RollDeg * dRoll, dt); break;
                    case Ch.SlipAngle:
                        if (!b.CpOk) continue;
                        for (var w = 0; w < 4; w++)
                        {
                            // The angle between where the tyre points (halfway
                            // through the block) and where its patch went, as
                            // the main line works out plain AC's.
                            var dx = b.Cx1[w] - b.Cx0[w]; var dz = b.Cz1[w] - b.Cz0[w];
                            var hx = b.Hx0[w] + b.Hx1[w]; var hz = b.Hz0[w] + b.Hz1[w];
                            if (Math.Sqrt(dx * dx + dz * dz) < 0.2 || (hx == 0 && hz == 0)) continue;
                            var a = Math.Atan2(Math.Abs(hx * dz - hz * dx), hx * dx + hz * dz) * Deg;
                            if (a > 90.0) a = 180.0 - a;   // rolling backwards: the same slip, mirrored
                            var page = b.SSlip[w] / n * Deg;
                            if (a > 45) continue;   // a spin: the patches say little
                            k.Fit.Add(page, a, dt / 4);
                        }
                        break;
                }
                k.Driven += dt;
                k.Pct = Math.Min(100, (int)(k.Driven * 100 / CheckMinSec));
                Decide(t, k);
            }
        }

        private void Decide(KunosTitle t, Check k)
        {
            if (k.Driven < k.NextDecide) return;
            k.NextDecide = k.Driven + 1.0;
            k.Still = k.Fit.N < 100 || Math.Sqrt(k.Fit.Vh) < k.Floor;
            if (k.Still) return;   // never judged on a page that barely moves
            var r = k.Fit.R; var ratio = k.Fit.K;
            if (k.SizeOnly) { r = Math.Abs(r); ratio = Math.Abs(ratio); }
            var why = "r " + r.ToString("0.00", CultureInfo.InvariantCulture) + ", x" + ratio.ToString("0.00", CultureInfo.InvariantCulture)
                    + (k.Loads != null ? ", " + k.LoadsWhat + " r " + k.Loads.R.ToString("0.00", CultureInfo.InvariantCulture) : "");
            if (k.Loads != null && k.Loads.N >= 100 && k.Loads.R <= -SwayLoadsMin)
            { Fail(t, k, "the other way to the side the load moves to, by " + k.LoadsWhat + " (" + why + ")"); return; }
            if (r <= -k.RMin) { Fail(t, k, "the other way to " + k.Against + " (" + why + ")"); return; }
            if (r >= k.RMin && (ratio < k.KLo || ratio > k.KHi)) { Fail(t, k, "at another scale than " + k.Against + ": another unit or axis (" + why + ")"); return; }
            var loadsOk = k.Loads == null || (k.Loads.N >= 100 && k.Loads.R >= SwayLoadsMin);
            if (r >= k.RMin && loadsOk)
            {
                k.Note = why;
                k.State = 1;
                _log("nullCAT Channel Exporter: Kunos direct, " + t.Name + ": " + k.Token + " from the page from now on (against "
                     + k.Against + ": " + why + ")");
                return;
            }
            if (k.Driven >= CheckGiveUpSec) Fail(t, k, "apart from " + k.Against + " (" + why + ")");
        }

        private void Fail(KunosTitle t, Check k, string why)
        {
            k.Note = why;
            k.State = -1;
            _log("nullCAT Channel Exporter: Kunos direct, " + t.Name + ": " + k.Token + " stays SimHub's this session: the page's runs " + why);
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
