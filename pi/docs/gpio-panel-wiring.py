#!/usr/bin/env python3
# ============================================================
# gpio-panel-wiring.py  --  generates gpio-panel-wiring.pdf
#
# Stdlib-only PDF generator (no reportlab/latex needed) for the nullCAT-Pi
# GPIO control-panel wiring reference. Regenerate with:
#     python3 gpio-panel-wiring.py
# ============================================================

W, H, M = 595.0, 842.0, 42.0   # A4 points, margin

def esc(s): return s.replace("\\", "\\\\").replace("(", "\\(").replace(")", "\\)")

class C:
    def __init__(self): self.o = []
    def text(self, x, y, f, s, t, col=(0, 0, 0)):
        r, g, b = col
        self.o.append(f"{r} {g} {b} rg BT /{f} {s} Tf 1 0 0 1 {x:.1f} {y:.1f} Tm ({esc(t)}) Tj ET")
    def line(self, x1, y1, x2, y2, w=0.6, col=(0.3, 0.3, 0.3)):
        r, g, b = col
        self.o.append(f"{r} {g} {b} RG {w} w {x1:.1f} {y1:.1f} m {x2:.1f} {y2:.1f} l S")
    def rect(self, x, y, w, h, lw=0.8, col=(0, 0, 0)):
        r, g, b = col
        self.o.append(f"{r} {g} {b} RG {lw} w {x:.1f} {y:.1f} {w:.1f} {h:.1f} re S")
    def fill(self, x, y, w, h, col):
        r, g, b = col
        self.o.append(f"{r} {g} {b} rg {x:.1f} {y:.1f} {w:.1f} {h:.1f} re f")
    def circ(self, cx, cy, rad, stroke=None, fillc=None, lw=0.8):
        k = 0.5523 * rad
        p = (f"{cx+rad:.1f} {cy:.1f} m "
             f"{cx+rad:.1f} {cy+k:.1f} {cx+k:.1f} {cy+rad:.1f} {cx:.1f} {cy+rad:.1f} c "
             f"{cx-k:.1f} {cy+rad:.1f} {cx-rad:.1f} {cy+k:.1f} {cx-rad:.1f} {cy:.1f} c "
             f"{cx-rad:.1f} {cy-k:.1f} {cx-k:.1f} {cy-rad:.1f} {cx:.1f} {cy-rad:.1f} c "
             f"{cx+k:.1f} {cy-rad:.1f} {cx+rad:.1f} {cy-k:.1f} {cx+rad:.1f} {cy:.1f} c ")
        if fillc:
            r, g, b = fillc; self.o.append(f"{r} {g} {b} rg " + p + "f")
        if stroke:
            r, g, b = stroke; self.o.append(f"{r} {g} {b} RG {lw} w " + p + "S")
    def stream(self): return ("\n".join(self.o)).encode("latin-1")

GREY = (0.45, 0.45, 0.45)
RED, GRN, AMB, BLU = (0.80, 0.12, 0.12), (0.12, 0.55, 0.18), (0.85, 0.55, 0.05), (0.10, 0.35, 0.75)

# -------------------- Page 1 --------------------
p1 = C()
y = H - M
p1.text(M, y, "F2", 19, "nullCAT-Pi  -  GPIO Control Panel"); y -= 18
p1.text(M, y, "F1", 11, "Wiring & pin reference", GREY); y -= 26
for ln in [
    "One Cat5e/Cat6 cable carries every signal: 7 of 8 conductors used.",
    "RJ45 / keystone breakout at each end.  NOT a network link - do not plug into a NIC.",
    "3.3 V logic only.  Buttons use internal pull-ups; LEDs are sourced by the GPIO pins.",
]:
    p1.text(M, y, "F1", 10, ln); y -= 14
y -= 12

# ---- Conductor map table ----
p1.text(M, y, "F2", 13, "Conductor map"); y -= 22
cols = [M, M+105, M+157, M+193, M+271]
hdr = ["Function", "BCM", "Phys", "Cat5e colour", "Notes"]
box_top = y                                       # top rule == top of the table box
p1.line(M, box_top, W-M, box_top, 0.8, (0, 0, 0))
y -= 14                                           # header baseline, clear below the rule
for i, htxt in enumerate(hdr):
    p1.text(cols[i], y, "F2", 9.5, htxt)
y -= 7
p1.line(M, y, W-M, y, 0.8, (0, 0, 0)); y -= 15
rows = [
    ("E-STOP signal", "GPIO17", "11", "white-orange", "NC mushroom to GND; open=stop"),
    ("Common GND",    "GND",    "14", "orange",       "shared return (one common wire)"),
    ("ENGAGE",        "GPIO27", "13", "white-green",  "momentary to GND"),
    ("PARK",          "GPIO22", "15", "green",        "momentary to GND"),
    ("LED Run (grn)", "GPIO23", "16", "white-blue",   "pin -> 330R -> LED -> GND"),
    ("LED Ready(amb)","GPIO24", "18", "blue",         "pin -> 330R -> LED -> GND"),
    ("LED Fault(red)","GPIO25", "22", "white-brown",  "pin -> 330R -> LED -> GND"),
    ("spare",         "-",      "-",  "brown",        "future 4th LED/button or 2nd GND"),
]
for idx, r in enumerate(rows):
    for i, cell in enumerate(r):
        p1.text(cols[i], y, "F3", 8.2, cell)
    if idx < len(rows) - 1:                       # faint separators BETWEEN rows only
        p1.line(M, y-4, W-M, y-4, 0.3, (0.82, 0.82, 0.82))
    y -= 15
box_bottom = y + 6                                # just below the last row
p1.rect(M, box_bottom, W-2*M, box_top - box_bottom, 0.8, (0, 0, 0))
y = box_bottom - 16

# ---- Behaviour ----
p1.text(M, y, "F2", 13, "Panel behaviour"); y -= 16
beh = [
    ("ENGAGE", "two-stage:  offline -> Initialize EtherCAT ;  ready (OP, stopped) -> Start loop"),
    ("PARK",   "running -> Park (returns platform to neutral; loop keeps running)"),
    ("E-STOP", "open (or cable cut) -> disable drives.  Release the mushroom"),
    ("",       "to clear the e-stop and the UI (auto-release)."),
]
for k, v in beh:
    if k: p1.text(M, y, "F2", 9.5, k)
    p1.text(M+62, y, "F1", 9.5, v); y -= 14
y -= 8
p1.text(M, y, "F2", 13, "Status LEDs"); y -= 16
leds = [
    (GRN, "Green",  "solid = control loop running"),
    (AMB, "Amber",  "solid = ready (OP, stopped) ;  blink = initializing"),
    (RED, "Red",    "solid = E-STOP ;  blink = drive fault"),
    ((0.6,0.6,0.6), "All off", "offline (EtherCAT not operational)"),
]
for col, name, desc in leds:
    p1.circ(M+5, y+3, 4, fillc=col)
    p1.text(M+16, y, "F2", 9.5, name)
    p1.text(M+70, y, "F1", 9.5, desc); y -= 15

p1.text(M, M-6, "F1", 8, "nullCAT-Pi GPIO panel  -  page 1/2", GREY)

# -------------------- Page 2 --------------------
p2 = C()
y = H - M
p2.text(M, y, "F2", 16, "Pi 40-pin header  -  block used (pins 11-22)"); y -= 16
p2.text(M, y, "F1", 9.5, "Odd pins on the top rail, even on the bottom (Pi 4, gpiochip0).", GREY); y -= 22

# header grid: 2 rows x 6 cols (pins 11..22)
cells_top = [("11","GPIO17","E-STOP",(1,0.9,0.9)), ("13","GPIO27","ENGAGE",(0.9,1,0.9)),
             ("15","GPIO22","PARK",(0.9,1,0.9)), ("17","3V3","do NOT use",(1,1,0.8)),
             ("19","GPIO10","unused",(0.95,0.95,0.95)), ("21","GPIO9","unused",(0.95,0.95,0.95))]
cells_bot = [("12","GPIO18","unused",(0.95,0.95,0.95)), ("14","GND","COMMON",(0.88,0.93,1)),
             ("16","GPIO23","LED Run",(0.9,1,0.9)), ("18","GPIO24","LED Ready",(0.9,1,0.9)),
             ("20","GND","spare GND",(0.95,0.95,0.95)), ("22","GPIO25","LED Fault",(0.9,1,0.9))]
cw, ch, gx = 82.0, 40.0, M
gridtop = y
for row, cells in [(0, cells_top), (1, cells_bot)]:
    cy = gridtop - row*(ch+6)
    for i, (pin, gpio, fn, bg) in enumerate(cells):
        x = gx + i*(cw+1)
        p2.fill(x, cy-ch, cw, ch, bg)
        p2.rect(x, cy-ch, cw, ch, 0.7, (0.3,0.3,0.3))
        p2.text(x+4, cy-12, "F2", 8.5, "pin "+pin)
        p2.text(x+4, cy-23, "F3", 8, gpio)
        p2.text(x+4, cy-34, "F1", 7.8, fn)
y = gridtop - 2*(ch+6) - 20
p2.text(M, y, "F1", 8.5, "Tip: a 2x6 IDC/keyed block over pins 11-22 grabs all 7 conductors at once (leave pin 17 = 3V3 unwired).", GREY)
y -= 26

# Panel face sketch
p2.text(M, y, "F2", 13, "Panel face layout (example)"); y -= 14
bx, by, bw, bh = M, y-150, W-2*M, 150
p2.rect(bx, by, bw, bh, 1.0, (0.2,0.2,0.2))
# mushroom
p2.circ(bx+70, by+bh-55, 26, stroke=RED, lw=2.0)
p2.circ(bx+70, by+bh-55, 14, fillc=RED)
p2.text(bx+45, by+bh-95, "F2", 9, "E-STOP")
# buttons
p2.circ(bx+200, by+bh-55, 16, stroke=(0.2,0.2,0.2), lw=1.4)
p2.text(bx+182, by+bh-95, "F2", 9, "ENGAGE")
p2.circ(bx+300, by+bh-55, 16, stroke=(0.2,0.2,0.2), lw=1.4)
p2.text(bx+288, by+bh-95, "F2", 9, "PARK")
# LEDs
for i,(col,lbl) in enumerate([(GRN,"RUN"),(AMB,"READY"),(RED,"FAULT")]):
    lx = bx+400 + i*42
    p2.circ(lx, by+bh-50, 7, fillc=col, stroke=(0.2,0.2,0.2), lw=0.6)
    p2.text(lx-13, by+bh-72, "F1", 7.5, lbl)
y = by - 18

# Notes
p2.text(M, y, "F2", 13, "Connectors & notes"); y -= 16
notes = [
    "Firmer than loose Dupont jumpers (in order): (1) screw-terminal GPIO HAT - wire each",
    "    conductor into a labelled terminal, clamps the whole header; (2) 40-pin IDC ribbon +",
    "    cobbler breakout; (3) a single 2x6 keyed crimp housing over pins 11-22.",
    "E-STOP is NC + fail-safe: closed=OK (low), open OR a cut/unplugged cable = stop (high).",
    "No separate power rail: buttons pull to the common GND; LEDs are driven by the GPIO pins.",
    "Pins are config-driven (gpio* keys in config.json) - reassign freely and tell the firmware.",
    "Enable in config.json: \"gpioEnabled\": true   (build needs libgpiod-dev; user in 'gpio' group).",
]
for ln in notes:
    p2.text(M, y, "F1", 9, ln); y -= 13.5

p2.text(M, M-6, "F1", 8, "nullCAT-Pi GPIO panel  -  page 2/2", GREY)

# -------------------- assemble PDF --------------------
objs = []
def add(b): objs.append(b); return len(objs)

c1, c2 = p1.stream(), p2.stream()
catalog = b"<< /Type /Catalog /Pages 2 0 R >>"
pages   = b"<< /Type /Pages /Kids [3 0 R 4 0 R] /Count 2 >>"
res     = b"/Resources << /Font << /F1 7 0 R /F2 8 0 R /F3 9 0 R >> >>"
page1   = b"<< /Type /Page /Parent 2 0 R /MediaBox [0 0 595 842] " + res + b" /Contents 5 0 R >>"
page2   = b"<< /Type /Page /Parent 2 0 R /MediaBox [0 0 595 842] " + res + b" /Contents 6 0 R >>"
cont1   = b"<< /Length %d >>\nstream\n" % len(c1) + c1 + b"\nendstream"
cont2   = b"<< /Length %d >>\nstream\n" % len(c2) + c2 + b"\nendstream"
f1 = b"<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica >>"
f2 = b"<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica-Bold >>"
f3 = b"<< /Type /Font /Subtype /Type1 /BaseFont /Courier >>"
for o in [catalog, pages, page1, page2, cont1, cont2, f1, f2, f3]:
    add(o)

out = bytearray(b"%PDF-1.4\n%\xe2\xe3\xcf\xd3\n")
offsets = []
for i, o in enumerate(objs, start=1):
    offsets.append(len(out))
    out += (b"%d 0 obj\n" % i) + o + b"\nendobj\n"
xref_pos = len(out)
n = len(objs) + 1
out += b"xref\n0 %d\n" % n
out += b"0000000000 65535 f \n"
for off in offsets:
    out += b"%010d 00000 n \n" % off
out += b"trailer\n<< /Size %d /Root 1 0 R >>\nstartxref\n%d\n%%%%EOF\n" % (n, xref_pos)

with open("gpio-panel-wiring.pdf", "wb") as fh:
    fh.write(out)
print("wrote gpio-panel-wiring.pdf (%d bytes, %d objects)" % (len(out), len(objs)))
